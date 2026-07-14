#!/usr/bin/env python3
"""
dashboard_node.py - Panel de monitoreo Gran Prix CapyTown - El Qhapaq Nan -
El Laberinto del Chaski - Grupo 4  (VERSION LABERINTO EN VIVO)
-------------------------------------------------------------------------------
Archivo NUEVO e INDEPENDIENTE. No modifica maze_solver.py, pare_detector.py ni
box_detector.py. Solo ESCUCHA los mismos topicos que ya existen (no publica
/cmd_vel ni nada que mueva al robot).

Abre UNA VENTANA nativa con OpenCV (cv2.imshow), igual que pare_detector.py.
No usa navegador, ni Flask, ni internet: solo cv2/numpy/rclpy.

QUE CAMBIO EN ESTA VERSION (pedido del grupo):
  * Se ELIMINARON los dos cuadros de la derecha (trayectoria en linea recta +
    grafico /lane_error vs tiempo).
  * En su lugar se dibuja EL LABERINTO REAL (rejilla 6x4, 360x240 cm, paredes
    MDF) y DENTRO se pinta EN VIVO el recorrido del carrito (posicion, rumbo y
    estela) a partir de /odom_raw.
  * Las metricas del reto se muestran abajo en TARJETAS ordenadas y legibles
    (Ronda, Ruta/eficiencia, Señales PARE, Integridad) en vez de dos renglones
    apretados.

DISTRIBUCION DE LA PANTALLA:
  [ Cabecera: titulo + grupo + ronda ]
  [ Camara (HSV + PARE) ]  [        LABERINTO EN VIVO         ]
  [ Tarjetas de metricas: Ronda | Ruta | PARE | Integridad     ]

Como correrlo (con /image_raw, /cmd_vel, /odom_raw y /color_detectado activos,
y DISPLAY apuntando a una pantalla real):
    export DISPLAY=:0
    python3 dashboard_node.py
Teclas: "q"/Esc cierra · "g" guarda una fila de metricas · "f" marca falso PARE.
"""

import csv
import math
import os
import random
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge

# =======================================================================
# CALIBRACION LiDAR (misma que maze_solver.py, SOLO para leer distancias
# frontal/izq/der y mostrarlas; no se importa ni se altera ese archivo)
# =======================================================================
FRONT_OFFSET   = 180
SECTOR_HALF    = 12
FOLLOW_SIDE    = "left"
TARGET_DIST    = 0.18
PARE_ROJO_SEGUNDOS = 3.0

TOPIC_IMAGEN   = "/image_raw"
TOPIC_COLOR    = "/color_detectado"
TOPIC_CMDVEL   = "/cmd_vel"
TOPIC_ODOM     = "/odom_raw"
TOPIC_SCAN     = "/scan"
TOPIC_RONDA      = "/ronda"              # String: "1" (exploracion) o "2" (time attack)
TOPIC_COLISION   = "/colision"           # Bool
TOPIC_DEADEND    = "/dead_end_evento"    # Bool
TOPIC_KARPINCHU  = "/karpinchu_rodeado"  # Bool

# =======================================================================
# GEOMETRIA DEL LABERINTO  ---  El Laberinto del Chaski (Escenario D)
# Rejilla 6 x 4, celdas de 60 cm  ->  360 x 240 cm.
# Origen (0,0) = esquina INFERIOR IZQUIERDA.  gx crece hacia la derecha,
# gy crece hacia arriba.  Una pared se define por dos vertices de rejilla
# (gx,gy) con gx en [0..6] y gy en [0..4].
#
#  >>> AJUSTA ESTAS LISTAS para calzar con el tablero fisico de cada ronda.
#      (las paredes internas y los PARE se reconfiguran entre corridas).
# =======================================================================
MAZE_COLS, MAZE_ROWS = 6, 4
CELL_M = 0.60                 # 60 cm por celda (para pasar odometria -> celdas)

INICIO_CELDA = (0, 0)         # esquina inferior izquierda
META_CELDA   = (5, 3)         # esquina superior derecha

# --- Paredes de PERIMETRO con sus dos aberturas (entrada izq. / salida der.) ---
PAREDES_PERIMETRO = [
    ((0, 0), (6, 0)),          # borde inferior completo
    ((0, 4), (6, 4)),          # borde superior completo
    ((0, 1), (0, 4)),          # borde izquierdo (deja ABIERTA la entrada en fila 0)
    ((6, 0), (6, 3)),          # borde derecho  (deja ABIERTA la salida  en fila 3)
]

# --- Paredes INTERNAS completas (tabla MDF de 60 cm) ---
#     [extraidas de la foto real del tablero - ronda actual] ---
PAREDES_INTERNAS = [
    ((1, 1), (1, 2)),          # V vertical medio-izquierda
    ((2, 3), (2, 4)),          # V vertical arriba-izquierda hacia el borde superior
    ((3, 0), (3, 1)),          # V vertical inferior-centro
    ((3, 2), (3, 3)),          # V vertical centro-arriba
    ((4, 1), (4, 2)),          # V vertical central
    ((5, 2), (5, 3)),          # V vertical junto a META
    ((4, 1), (6, 1)),          # H horizontal inferior-derecha
    ((2, 2), (4, 2)),          # H horizontal central larga
    ((1, 3), (2, 3)),          # H horizontal medio-izquierda (arriba)
    ((4, 3), (5, 3)),          # H horizontal derecha-arriba
]

# --- Medias tablas / stubs de 30 cm (chicanes) ---
PAREDES_STUBS = [
    ((2, 0.0), (2, 0.55)),        # stub vertical abajo-centro (media tabla)
    ((4.45, 1.5), (5.0, 1.5)),    # stub horizontal centro-derecha (media tabla)
]

# --- Señales PARE colgadas: celda (col,row) donde hay un PARE.  Cambian cada ronda ---
PARE_CELDAS = [(3, 3), (1, 2), (2, 1)]

# --- Ruta sugerida (mas corta, 1 de varias) para dibujar punteada. Editable/off ---
MOSTRAR_RUTA_SUGERIDA = False   # <- ruta azul punteada desactivada
RUTA_SUGERIDA = [(0.5, 0.5), (0.5, 2.5), (3.5, 2.5), (3.5, 3.5), (5.5, 3.5)]

# =======================================================================
# ODOMETRIA -> CELDAS DEL LABERINTO
# El robot arranca en el centro de la celda INICIO. Ajusta los signos/ejes
# segun como quede montado el robot al entrar (mirando hacia +x del laberinto).
# =======================================================================
START_GX, START_GY = INICIO_CELDA[0] + 0.5, INICIO_CELDA[1] + 0.5
ODOM_SWAP = True     # robot arranca mirando HACIA ARRIBA (+y): adelante = subir
ODOM_FX   = -1.0     # signo lateral (si el recorrido sale espejado izq/der, cambia a +1.0)
ODOM_FY   = +1.0     # signo de avance (adelante = arriba)
ODOM_ESCALA = 0.8    # <1.0 ENCOGE el recorrido, >1.0 lo agranda (NO altera la orientacion).
                     # Si la estela se sale del laberinto, baja este valor hasta que quepa.
                     # Ej.: si el trazo sale ~30% mas grande, usa 1/1.30 ~= 0.77

PARE_REALES    = 3          # cuantas señales PARE hay en la pista (cambia por corrida)
LONG_OPTIMA_CM = 420.0      # ruta mas corta del trazado (referencia; ajustar)
METRICAS_CSV   = "metricas_granprix.csv"
EQUIPO_NOMBRE  = "Grupo 4"

WINDOW_NAME    = f"Gran Prix CapyTown - El Laberinto del Chaski - {EQUIPO_NOMBRE}"
CANVAS_W, CANVAS_H = 1440, 900
FPS_RENDER     = 15

HSV_AMARILLO = (np.array([18, 80, 90]),  np.array([35, 255, 255]))
HSV_BLANCO   = (np.array([0, 0, 200]),   np.array([180, 40, 255]))

# ---- Paleta (BGR) estilo "pit wall": carbon + acentos de carrera ----
C_BG    = (18, 15, 12)
C_PANEL = (30, 25, 21)
C_CARD  = (40, 33, 28)
C_LINE  = (60, 51, 43)
C_TEXT  = (240, 243, 240)
C_MUTED = (168, 152, 141)
C_RED   = (60, 64, 235)
C_GREEN = (110, 205, 120)
C_AMBER = (24, 156, 246)
C_YELLOW= (72, 214, 255)
C_WALL  = (14, 86, 181)     # marron/naranja quemado de las tablas MDF
C_STUB  = (26, 128, 214)    # medias tablas, un poco mas claras
C_FLOOR = (26, 22, 19)
C_GRID  = (52, 45, 38)
C_ROUTE = (150, 165, 45)    # teal punteado (ruta sugerida)
C_TRAIL = (72, 214, 255)    # estela del robot
C_START = (70, 140, 80)
C_META  = (120, 210, 130)

F_SIMPLE = cv2.FONT_HERSHEY_SIMPLEX
F_DUPLEX = cv2.FONT_HERSHEY_DUPLEX


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def odom_off_to_grid(dx, dy):
    """Convierte un desplazamiento (dx,dy) de odometria a ejes del laberinto.
    Usa las mismas perillas ODOM_SWAP/FX/FY para posicion Y para el rumbo."""
    ox, oy = dx, dy
    if ODOM_SWAP:
        ox, oy = oy, ox
    return (ox * ODOM_FX, oy * ODOM_FY)


def put_text(img, text, org, scale=0.6, color=C_TEXT, thickness=1, font=F_SIMPLE):
    cv2.putText(img, text, org, font, scale, color, thickness, cv2.LINE_AA)


def text_w(text, scale=0.6, thickness=1, font=F_SIMPLE):
    return cv2.getTextSize(text, font, scale, thickness)[0][0]


def rounded_rect(img, x1, y1, x2, y2, r, color, thickness=-1):
    r = max(1, min(r, (x2 - x1) // 2, (y2 - y1) // 2))
    if thickness < 0:
        cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1)
        cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r), color, -1)
        for cx, cy in ((x1 + r, y1 + r), (x2 - r, y1 + r), (x1 + r, y2 - r), (x2 - r, y2 - r)):
            cv2.circle(img, (cx, cy), r, color, -1)
    else:
        cv2.line(img, (x1 + r, y1), (x2 - r, y1), color, thickness, cv2.LINE_AA)
        cv2.line(img, (x1 + r, y2), (x2 - r, y2), color, thickness, cv2.LINE_AA)
        cv2.line(img, (x1, y1 + r), (x1, y2 - r), color, thickness, cv2.LINE_AA)
        cv2.line(img, (x2, y1 + r), (x2, y2 - r), color, thickness, cv2.LINE_AA)
        cv2.ellipse(img, (x1 + r, y1 + r), (r, r), 180, 0, 90, color, thickness, cv2.LINE_AA)
        cv2.ellipse(img, (x2 - r, y1 + r), (r, r), 270, 0, 90, color, thickness, cv2.LINE_AA)
        cv2.ellipse(img, (x1 + r, y2 - r), (r, r), 90, 0, 90, color, thickness, cv2.LINE_AA)
        cv2.ellipse(img, (x2 - r, y2 - r), (r, r), 0, 0, 90, color, thickness, cv2.LINE_AA)


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_node")
        self.bridge = CvBridge()

        # --- estado ---
        self.cam_frame = None
        self.color = "NINGUNO"
        self.color_edge_ts = None
        self.meta_reached = False

        self.lin = 0.0
        self.ang = 0.0
        self.speed_samples = deque(maxlen=400)
        self.comportamiento = "Detenido"
        self.giros = 0

        self.traj = deque(maxlen=6000)     # (gx,gy) en celdas del laberinto
        self.robot_gx = None
        self.robot_gy = None
        self.robot_yaw = 0.0
        self.start_xy = None
        self.away_from_start = False

        self.front_d = self.left_d = self.right_d = float("nan")
        self.t0 = time.time()
        self.confetti = []

        # --- metricas ---
        self.ronda = "1"
        self.tiempo_inicio_ronda = time.time()
        self._last_xy_dist = None
        self.long_ruta_cm = 0.0
        self.colisiones = 0
        self.dead_ends = 0
        self.karpinchus_rodeados = 0
        self.pare_detectados = 0
        self.pare_respetados = 0
        self.pare_falsos = 0
        self._fila_guardada_ronda = False
        self._csv_existe = os.path.isfile(METRICAS_CSV)

        self.create_subscription(Image, TOPIC_IMAGEN, self.on_image, 10)
        self.create_subscription(String, TOPIC_COLOR, self.on_color, 10)
        self.create_subscription(Twist, TOPIC_CMDVEL, self.on_cmdvel, 10)
        self.create_subscription(Odometry, TOPIC_ODOM, self.on_odom, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self.on_scan, 10)
        self.create_subscription(String, TOPIC_RONDA, self.on_ronda, 10)
        self.create_subscription(Bool, TOPIC_COLISION, self.on_colision, 10)
        self.create_subscription(Bool, TOPIC_DEADEND, self.on_deadend, 10)
        self.create_subscription(Bool, TOPIC_KARPINCHU, self.on_karpinchu, 10)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, CANVAS_W, CANVAS_H)
        self.get_logger().info("dashboard_node (laberinto en vivo) iniciado. Tecla 'q' para salir.")

    # --------------------------- Callbacks ROS ---------------------------
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m_y = cv2.inRange(hsv, *HSV_AMARILLO)
        m_w = cv2.inRange(hsv, *HSV_BLANCO)
        overlay = frame.copy()
        overlay[m_y > 0] = (0, 210, 255)
        overlay[m_w > 0] = (255, 255, 255)
        self.cam_frame = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    def on_color(self, msg):
        nuevo = msg.data
        if nuevo != self.color:
            if nuevo == "ROJO":
                self.color_edge_ts = time.time()
                self.pare_detectados += 1
            if self.color == "ROJO" and nuevo != "ROJO":
                duracion = time.time() - (self.color_edge_ts or time.time())
                if duracion >= PARE_ROJO_SEGUNDOS - 0.3:
                    self.pare_respetados += 1
            if nuevo == "VERDE":
                self.meta_reached = True
                self.spawn_confetti()
                if not self._fila_guardada_ronda:
                    self.guardar_metricas(llego_meta=True)
                    self._fila_guardada_ronda = True
        self.color = nuevo

    def on_cmdvel(self, msg):
        self.lin = msg.linear.x
        self.ang = msg.angular.z
        self.speed_samples.append(abs(msg.linear.x))
        anterior = self.comportamiento
        self.comportamiento = self.clasificar_comportamiento(self.lin, self.ang)
        girando_ahora = self.comportamiento in ("Girando a la izquierda", "Girando a la derecha")
        girando_antes = anterior in ("Girando a la izquierda", "Girando a la derecha")
        if girando_ahora and not girando_antes:
            self.giros += 1

    @staticmethod
    def clasificar_comportamiento(lin, ang):
        if abs(ang) < 0.05 and abs(lin) > 0.01:
            return "Avanzando en linea recta"
        elif abs(lin) < 0.01 and abs(ang) < 0.01:
            return "Detenido"
        elif ang > 0:
            return "Girando a la izquierda"
        else:
            return "Girando a la derecha"

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quat(msg.pose.pose.orientation)

        # odometria (m) -> celdas del laberinto
        offx, offy = odom_off_to_grid(x, y)
        gx = START_GX + (offx / CELL_M) * ODOM_ESCALA
        gy = START_GY + (offy / CELL_M) * ODOM_ESCALA
        self.robot_gx, self.robot_gy = gx, gy
        self.traj.append((gx, gy))

        if self.start_xy is None:
            self.start_xy = (x, y)
        if self._last_xy_dist is not None:
            paso = math.hypot(x - self._last_xy_dist[0], y - self._last_xy_dist[1])
            self.long_ruta_cm += paso * 100.0
        self._last_xy_dist = (x, y)

    def on_ronda(self, msg):
        if msg.data != self.ronda:
            self.ronda = msg.data
            self.reiniciar_metricas_ronda()

    def on_colision(self, msg):
        if msg.data:
            self.colisiones += 1

    def on_deadend(self, msg):
        if msg.data:
            self.dead_ends += 1

    def on_karpinchu(self, msg):
        if msg.data:
            self.karpinchus_rodeados += 1

    def on_scan(self, scan):
        """Lee distancia frontal / izquierda / derecha (informativo, fusion LiDAR)."""
        n = len(scan.ranges)
        if n == 0:
            return

        def sector_min(center_deg, half=SECTOR_HALF):
            vals = []
            for d in range(-half, half + 1):
                ang = math.radians(center_deg + FRONT_OFFSET + d)
                idx = int(round((ang - scan.angle_min) / scan.angle_increment)) % n
                r = scan.ranges[idx]
                if r is None or math.isinf(r) or math.isnan(r):
                    continue
                if r <= scan.range_min or r > scan.range_max:
                    continue
                vals.append(r)
            return min(vals) if vals else float("nan")

        self.front_d = sector_min(0)
        self.left_d = sector_min(90)
        self.right_d = sector_min(-90)

    # --------------------------- Metricas ---------------------------
    def reiniciar_metricas_ronda(self):
        self.tiempo_inicio_ronda = time.time()
        self.long_ruta_cm = 0.0
        self._last_xy_dist = None
        self.start_xy = None
        self.colisiones = 0
        self.dead_ends = 0
        self.karpinchus_rodeados = 0
        self.pare_detectados = 0
        self.pare_respetados = 0
        self.pare_falsos = 0
        self.giros = 0
        self.meta_reached = False
        self._fila_guardada_ronda = False
        self.traj.clear()

    def guardar_metricas(self, llego_meta):
        tiempo_s = time.time() - self.tiempo_inicio_ronda
        eficiencia = (LONG_OPTIMA_CM / self.long_ruta_cm) if self.long_ruta_cm > 0 else 0.0
        fila = {
            "ronda": self.ronda,
            "llego_meta": "Si" if llego_meta else "No",
            "tiempo_s": f"{tiempo_s:.1f}",
            "long_ruta_cm": f"{self.long_ruta_cm:.1f}",
            "long_optima_cm": f"{LONG_OPTIMA_CM:.1f}",
            "eficiencia": f"{eficiencia:.2f}",
            "colisiones": self.colisiones,
            "pare_reales": PARE_REALES,
            "pare_detectados": self.pare_detectados,
            "pare_respetados": self.pare_respetados,
            "pare_falsos": self.pare_falsos,
            "dead_ends_visitados": self.dead_ends,
            "karpinchus_rodeados": self.karpinchus_rodeados,
        }
        try:
            with open(METRICAS_CSV, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(fila.keys()))
                if not self._csv_existe:
                    writer.writeheader()
                    self._csv_existe = True
                writer.writerow(fila)
            self.get_logger().info(f"Metricas guardadas en {METRICAS_CSV}: {fila}")
        except Exception as e:
            self.get_logger().error(f"No se pudo guardar {METRICAS_CSV}: {e}")

    # --------------------------- Confeti ---------------------------
    def spawn_confetti(self, n=90):
        colors = [C_RED, C_GREEN, C_AMBER, C_YELLOW, (255, 255, 255)]
        self.confetti = [{
            "x": random.uniform(0, 1), "y": random.uniform(-0.6, 0),
            "vy": random.uniform(0.01, 0.02), "vx": random.uniform(-0.006, 0.006),
            "c": random.choice(colors), "r": random.randint(3, 6),
        } for _ in range(n)]

    def step_confetti(self, w, h, img):
        for p in self.confetti:
            p["y"] += p["vy"]
            p["x"] += p["vx"]
            if p["y"] > 1.05:
                p["y"] = -0.05
                p["x"] = random.uniform(0, 1)
            cv2.rectangle(img,
                          (int(p["x"] * w) - p["r"], int(p["y"] * h) - p["r"]),
                          (int(p["x"] * w) + p["r"], int(p["y"] * h) + p["r"]),
                          p["c"], -1)

    # --------------------------- Dibujo: paneles ---------------------------
    def draw_panel(self, img, x, y, w, h, title, accent=C_AMBER):
        rounded_rect(img, x, y, x + w, y + h, 12, C_PANEL, -1)
        rounded_rect(img, x, y, x + w, y + h, 12, C_LINE, 1)
        cv2.rectangle(img, (x + 14, y + 16), (x + 18, y + 30), accent, -1)
        put_text(img, title, (x + 26, y + 29), 0.52, C_TEXT, 1, F_DUPLEX)
        return (x + 12, y + 42, w - 24, h - 54)

    # --------------------------- Dibujo: LABERINTO ---------------------------
    def draw_maze(self, img, x, y, w, h):
        cell = min(w / MAZE_COLS, h / MAZE_ROWS) * 1.10   # mapa 10% mas grande
        mw, mh = cell * MAZE_COLS, cell * MAZE_ROWS
        ox = x + (w - mw) / 2.0
        oy = y + (h - mh) / 2.0

        def gp(gx, gy):
            return (int(round(ox + gx * cell)), int(round(oy + (MAZE_ROWS - gy) * cell)))

        # piso
        cv2.rectangle(img, gp(0, MAZE_ROWS), gp(MAZE_COLS, 0), C_FLOOR, -1)
        for c in range(MAZE_COLS + 1):
            cv2.line(img, gp(c, 0), gp(c, MAZE_ROWS), C_GRID, 1, cv2.LINE_AA)
        for r in range(MAZE_ROWS + 1):
            cv2.line(img, gp(0, r), gp(MAZE_COLS, r), C_GRID, 1, cv2.LINE_AA)

        # celdas INICIO / META
        sc = INICIO_CELDA
        cv2.rectangle(img, gp(sc[0], sc[1] + 1), gp(sc[0] + 1, sc[1]), C_START, -1)
        mc = META_CELDA
        ov = img.copy()
        cv2.rectangle(ov, gp(mc[0], mc[1] + 1), gp(mc[0] + 1, mc[1]), C_META, -1)
        cv2.addWeighted(ov, 0.55, img, 0.45, 0, img)
        cv2.rectangle(img, gp(mc[0], mc[1] + 1), gp(mc[0] + 1, mc[1]), C_META, 2)

        # etiquetas entrada/salida
        put_text(img, "entrada", (int(ox - cell * 0.02), gp(0, 0)[1] + int(cell * 0.35)),
                 0.4, C_MUTED, 1)
        put_text(img, "salida", (gp(6, 3)[0] + 6, gp(6, 3)[1] + int(cell * 0.55)),
                 0.4, C_MUTED, 1)
        p = gp(sc[0], sc[1]); put_text(img, "INICIO", (p[0] + 6, p[1] - 8), 0.4, C_TEXT, 1)
        pm = gp(mc[0], mc[1] + 1); put_text(img, "META", (pm[0] + int(cell*0.28), pm[1] + int(cell*0.55)), 0.5, (25,60,25), 2)

        # ruta sugerida (punteada)
        if MOSTRAR_RUTA_SUGERIDA and len(RUTA_SUGERIDA) >= 2:
            pts = [gp(px, py) for (px, py) in RUTA_SUGERIDA]
            for i in range(1, len(pts)):
                self._dashed_line(img, pts[i - 1], pts[i], C_ROUTE, 2, dash=10)

        # paredes
        wt = max(3, int(cell * 0.11))
        for (a, b) in PAREDES_PERIMETRO + PAREDES_INTERNAS:
            cv2.line(img, gp(*a), gp(*b), C_WALL, wt, cv2.LINE_AA)
        for (a, b) in PAREDES_PERIMETRO + PAREDES_INTERNAS:   # remates redondeados
            cv2.circle(img, gp(*a), wt // 2, C_WALL, -1)
            cv2.circle(img, gp(*b), wt // 2, C_WALL, -1)
        for (a, b) in PAREDES_STUBS:
            cv2.line(img, gp(*a), gp(*b), C_STUB, max(3, int(wt * 0.8)), cv2.LINE_AA)

        # señales PARE
        for (cc, cr) in PARE_CELDAS:
            px, py = gp(cc + 0.5, cr + 0.85)
            cv2.circle(img, (px, py), max(9, int(cell * 0.16)), C_RED, -1)
            cv2.circle(img, (px, py), max(9, int(cell * 0.16)), (255, 255, 255), 1, cv2.LINE_AA)
            tw = text_w("PARE", 0.32, 1)
            put_text(img, "PARE", (px - tw // 2, py + 4), 0.32, (255, 255, 255), 1)

        # ------- ESTELA + ROBOT EN VIVO (recortado al area del laberinto) -------
        mrect = (int(ox), int(oy), int(mw), int(mh))
        pts = [gp(gx, gy) for (gx, gy) in self.traj]
        if len(pts) >= 2:
            npts = len(pts)
            for i in range(1, npts):
                inside, q1, q2 = cv2.clipLine(mrect, pts[i - 1], pts[i])
                if not inside:
                    continue
                f = i / npts                       # 0 (viejo) -> 1 (reciente)
                col = (int(C_TRAIL[0]), int(60 + 155 * f), int(120 + 135 * f))
                cv2.line(img, q1, q2, col, max(2, int(cell * 0.05)), cv2.LINE_AA)

        if self.robot_gx is not None:
            rp = gp(self.robot_gx, self.robot_gy)
            dentro = (mrect[0] <= rp[0] <= mrect[0] + mrect[2] and
                      mrect[1] <= rp[1] <= mrect[1] + mrect[3])
            if dentro:
                rr = max(6, int(cell * 0.15))
                cv2.circle(img, rp, rr + 3, (0, 0, 0), -1)
                cv2.circle(img, rp, rr, C_YELLOW, -1)
                cv2.circle(img, rp, rr, (20, 20, 20), 1, cv2.LINE_AA)
                # flecha de rumbo (misma transformacion que el recorrido)
                dgx, dgy = odom_off_to_grid(math.cos(self.robot_yaw), math.sin(self.robot_yaw))
                tip = (int(rp[0] + dgx * rr * 2.1), int(rp[1] - dgy * rr * 2.1))
                cv2.arrowedLine(img, rp, tip, (20, 20, 20), 3, cv2.LINE_AA, tipLength=0.4)

        # lectura LiDAR (fusion) en esquina
        def fmt(d):
            return f"{d:.2f}m" if not math.isnan(d) else "--"
        put_text(img, f"LiDAR  F:{fmt(self.front_d)}  I:{fmt(self.left_d)}  D:{fmt(self.right_d)}",
                 (int(ox) + 6, int(oy + mh) - 8), 0.42, C_MUTED, 1)

    def _dashed_line(self, img, p1, p2, color, thickness, dash=10):
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if dist < 1:
            return
        n = int(dist / dash)
        for i in range(0, n, 2):
            a = (int(p1[0] + (p2[0] - p1[0]) * i / n), int(p1[1] + (p2[1] - p1[1]) * i / n))
            b = (int(p1[0] + (p2[0] - p1[0]) * (i + 1) / n), int(p1[1] + (p2[1] - p1[1]) * (i + 1) / n))
            cv2.line(img, a, b, color, thickness, cv2.LINE_AA)

    # --------------------------- Dibujo: tarjetas metricas ----------------
    def draw_card(self, img, x, y, w, h, title, accent):
        rounded_rect(img, x, y, x + w, y + h, 10, C_CARD, -1)
        rounded_rect(img, x, y, x + w, y + h, 10, C_LINE, 1)
        cv2.rectangle(img, (x, y + 10), (x + 4, y + h - 10), accent, -1)
        put_text(img, title, (x + 16, y + 22), 0.44, accent, 1, F_DUPLEX)
        return (x + 16, y + 40)

    def stat(self, img, x, y, label, value, color=C_TEXT, big=0.62):
        put_text(img, value, (x, y), big, color, 1, F_DUPLEX)
        put_text(img, label, (x, y + 16), 0.38, C_MUTED, 1)

    # --------------------------- RENDER ---------------------------
    def render(self):
        img = np.full((CANVAS_H, CANVAS_W, 3), C_BG, dtype=np.uint8)
        pad = 18

        # ---- cabecera ----
        put_text(img, "GRAN PRIX CAPYTOWN", (24, 34), 0.82, C_TEXT, 2, F_DUPLEX)
        put_text(img, "El Qhapaq Nan - Laberinto del Chaski", (24, 56), 0.46, C_AMBER, 1, F_DUPLEX)
        sub = f"Yahboom Pi5 Robocar - Fusion Camara + LiDAR - {EQUIPO_NOMBRE}"
        put_text(img, sub, (CANVAS_W - text_w(sub, 0.44, 1) - 24, 34), 0.44, C_MUTED, 1)
        ronda_lbl = "RONDA 1 - Reconocimiento" if self.ronda == "1" else "RONDA 2 - Time Attack"
        rc = C_AMBER if self.ronda == "1" else C_YELLOW
        rw = text_w(ronda_lbl, 0.5, 1, F_DUPLEX) + 24
        rounded_rect(img, CANVAS_W - rw - 24, 44, CANVAS_W - 24, 68, 10, C_PANEL, -1)
        rounded_rect(img, CANVAS_W - rw - 24, 44, CANVAS_W - 24, 68, 10, rc, 1)
        put_text(img, ronda_lbl, (CANVAS_W - rw - 12, 61), 0.5, rc, 1, F_DUPLEX)
        cv2.line(img, (0, 78), (CANVAS_W, 78), C_LINE, 1)

        top = 90
        metrics_h = 166
        content_h = CANVAS_H - top - metrics_h - pad
        cam_w = 470
        maze_x = pad * 2 + cam_w
        maze_w = CANVAS_W - maze_x - pad

        # ---- panel camara ----
        cx, cy, cw, ch = self.draw_panel(img, pad, top, cam_w, content_h,
                                         "CAMARA - Segmentacion HSV & PARE", C_AMBER)
        img_h = int(cw * 0.72)
        if self.cam_frame is not None:
            img[cy:cy + img_h, cx:cx + cw] = cv2.resize(self.cam_frame, (cw, img_h))
        else:
            cv2.rectangle(img, (cx, cy), (cx + cw, cy + img_h), (10, 10, 10), -1)
            put_text(img, "Esperando /image_raw...", (cx + 14, cy + img_h // 2), 0.5, C_MUTED, 1)

        # chip Color Detectado
        if self.color == "ROJO":
            rem = max(0.0, PARE_ROJO_SEGUNDOS - (time.time() - (self.color_edge_ts or time.time())))
            ctxt, cc2 = f"ROJO - PARE, deteniendo {rem:.1f}s", C_RED
        elif self.color == "VERDE":
            ctxt, cc2 = "VERDE - META detectada!", C_GREEN
        else:
            ctxt, cc2 = "NINGUNO - Via libre", C_MUTED
        chy = cy + img_h + 12
        rounded_rect(img, cx, chy, cx + cw, chy + 34, 8, (12, 12, 12), -1)
        rounded_rect(img, cx, chy, cx + cw, chy + 34, 8, cc2, 2)
        cv2.circle(img, (cx + 18, chy + 17), 6, cc2, -1)
        put_text(img, "Color detectado:", (cx + 34, chy + 22), 0.44, C_MUTED, 1)
        put_text(img, ctxt, (cx + 34 + text_w("Color detectado: ", 0.44, 1), chy + 22), 0.46, C_TEXT, 1, F_DUPLEX)

        # nota de arbitraje / convencion (suma para la defensa)
        ny = chy + 48
        put_text(img, "REGLA DE ARBITRAJE (fusion)", (cx, ny), 0.42, C_YELLOW, 1, F_DUPLEX)
        for i, ln in enumerate([
            "Camara MANDA para DETENER (seguridad/regla).",
            "LiDAR MANDA para MOVER y centrar en el pasillo.",
            "PARE detectado -> PARAR ~3s aunque el pasillo este libre.",
            "ROJO = PARE (pausa 3s)   ·   VERDE = META (fin).",
        ]):
            put_text(img, ln, (cx, ny + 20 + i * 19), 0.4, C_MUTED, 1)

        estado = f"Estado: {self.comportamiento}"
        put_text(img, estado, (cx, cy + ch - 6), 0.44, C_TEXT, 1, F_DUPLEX)

        # ---- panel LABERINTO ----
        lx, ly, lw, lh = self.draw_panel(img, maze_x, top, maze_w, content_h,
                                         "LABERINTO EN VIVO - Recorrido del Karpinchu Chaski", C_GREEN)
        badge = f"{self.comportamiento}   ·   Giros: {self.giros}"
        bw = text_w(badge, 0.44, 1, F_DUPLEX) + 20
        rounded_rect(img, lx + lw - bw, ly - 2, lx + lw, ly + 22, 8, (20, 17, 14), -1)
        put_text(img, badge, (lx + lw - bw + 10, ly + 15), 0.44, C_YELLOW, 1, F_DUPLEX)
        self.draw_maze(img, lx, ly + 26, lw, lh - 26)

        # overlay META (confeti)
        if self.meta_reached:
            ov = img.copy()
            cv2.rectangle(ov, (maze_x, top), (maze_x + maze_w, top + content_h), (5, 5, 5), -1)
            img = cv2.addWeighted(ov, 0.55, img, 0.45, 0)
            self.step_confetti(maze_w, content_h, img[top:top + content_h, maze_x:maze_x + maze_w])
            m1 = "LLEGAMOS A LA HUACA - META"
            m2 = f"El Laberinto del Chaski - {EQUIPO_NOMBRE}!"
            cxm = maze_x + maze_w // 2
            cym = top + content_h // 2
            put_text(img, m1, (cxm - text_w(m1, 1.0, 2, F_DUPLEX) // 2, cym - 6), 1.0, C_AMBER, 2, F_DUPLEX)
            put_text(img, m2, (cxm - text_w(m2, 0.7, 2, F_DUPLEX) // 2, cym + 30), 0.7, C_YELLOW, 2, F_DUPLEX)

        # ---- tarjetas de metricas ----
        by0 = CANVAS_H - metrics_h
        cv2.line(img, (0, by0), (CANVAS_W, by0), C_LINE, 1)
        put_text(img, "METRICAS GRAN PRIX", (pad, by0 + 18), 0.46, C_TEXT, 1, F_DUPLEX)
        put_text(img, "Teclas:  q/Esc salir   ·   g guardar fila en metricas_granprix.csv   ·   f marcar falso PARE",
                 (pad + 240, by0 + 18), 0.4, C_MUTED, 1)

        card_y = by0 + 26
        card_h = metrics_h - 34
        gap = 14
        card_w = (CANVAS_W - 2 * pad - 3 * gap) // 4
        vel = (sum(self.speed_samples) / len(self.speed_samples)) if self.speed_samples else 0.0
        tiempo = time.time() - self.tiempo_inicio_ronda
        efi = (LONG_OPTIMA_CM / self.long_ruta_cm) if self.long_ruta_cm > 0 else 0.0

        # Card 1: Ronda / tiempo / estado
        x0 = pad
        ix, iy = self.draw_card(img, x0, card_y, card_w, card_h, "RONDA", C_AMBER)
        self.stat(img, ix, iy + 6, "Ronda", "1 - Exploracion" if self.ronda == "1" else "2 - Time Attack", C_TEXT, 0.5)
        self.stat(img, ix, iy + 46, "Tiempo (s)", f"{tiempo:5.1f}", C_YELLOW, 0.72)
        self.stat(img, ix + 150, iy + 46, "Vel. media", f"{vel:.2f} m/s", C_TEXT, 0.6)

        # Card 2: Ruta / eficiencia
        x0 += card_w + gap
        ix, iy = self.draw_card(img, x0, card_y, card_w, card_h, "RUTA", C_GREEN)
        self.stat(img, ix, iy + 6, "Recorrida", f"{self.long_ruta_cm:.0f} cm", C_TEXT, 0.58)
        self.stat(img, ix + 150, iy + 6, "Optima", f"{LONG_OPTIMA_CM:.0f} cm", C_MUTED, 0.58)
        efi_col = C_GREEN if efi >= 0.7 else (C_AMBER if efi >= 0.4 else C_RED)
        self.stat(img, ix, iy + 50, "Eficiencia", f"{efi:.2f}", efi_col, 0.72)
        # barra de eficiencia
        bx1, bx2 = ix + 120, x0 + card_w - 16
        byb = iy + 40
        rounded_rect(img, bx1, byb, bx2, byb + 14, 6, (22, 20, 18), -1)
        fillw = int((bx2 - bx1) * max(0.0, min(1.0, efi)))
        if fillw > 4:
            rounded_rect(img, bx1, byb, bx1 + fillw, byb + 14, 6, efi_col, -1)
        gx07 = bx1 + int((bx2 - bx1) * 0.7)
        cv2.line(img, (gx07, byb - 2), (gx07, byb + 16), C_TEXT, 1)
        put_text(img, "meta 0.70", (gx07 - 20, byb + 30), 0.34, C_MUTED, 1)

        # Card 3: Señales PARE
        x0 += card_w + gap
        ix, iy = self.draw_card(img, x0, card_y, card_w, card_h, "SEÑALES PARE", C_RED)
        self.stat(img, ix, iy + 6, "Reales", f"{PARE_REALES}", C_TEXT, 0.72)
        self.stat(img, ix + 100, iy + 6, "Detectadas", f"{self.pare_detectados}", C_AMBER, 0.72)
        resp_col = C_GREEN if self.pare_respetados >= PARE_REALES else C_AMBER
        self.stat(img, ix, iy + 50, "Respetadas 3s", f"{self.pare_respetados}", resp_col, 0.72)
        falso_col = C_RED if self.pare_falsos > 0 else C_GREEN
        self.stat(img, ix + 130, iy + 50, "Falsos", f"{self.pare_falsos}", falso_col, 0.72)

        # Card 4: Integridad
        x0 += card_w + gap
        ix, iy = self.draw_card(img, x0, card_y, card_w, card_h, "INTEGRIDAD", C_YELLOW)
        col_col = C_GREEN if self.colisiones == 0 else C_RED
        self.stat(img, ix, iy + 6, "Colisiones", f"{self.colisiones}", col_col, 0.72)
        self.stat(img, ix + 130, iy + 6, "Dead-ends", f"{self.dead_ends}", C_TEXT, 0.72)
        self.stat(img, ix, iy + 50, "Karpinchus", f"{self.karpinchus_rodeados}", C_TEXT, 0.72)
        meta_txt = "SI" if self.meta_reached else "--"
        self.stat(img, ix + 130, iy + 50, "Llego META", meta_txt,
                  C_GREEN if self.meta_reached else C_MUTED, 0.72)

        return img


def main():
    rclpy.init()
    node = DashboardNode()
    period = 1.0 / FPS_RENDER
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            canvas = node.render()
            cv2.imshow(WINDOW_NAME, canvas)
            key = cv2.waitKey(max(1, int(period * 1000))) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('g'):
                node.guardar_metricas(llego_meta=node.meta_reached)
            elif key == ord('f'):
                node.pare_falsos += 1
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()