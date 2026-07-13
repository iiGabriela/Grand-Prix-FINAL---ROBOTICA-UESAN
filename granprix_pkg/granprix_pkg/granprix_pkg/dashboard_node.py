#!/usr/bin/env python3
"""
dashboard_node.py - Panel de monitoreo Gran Prix CapyTown (Semana 11 · RC-2)
-------------------------------------------------------------------------------
Archivo NUEVO e INDEPENDIENTE. No modifica maze_solver.py ni pare_detector.py.
Solo ESCUCHA los mismos topicos que ya existen (no publica /cmd_vel ni nada
que mueva al robot).

Abre UNA VENTANA nativa con OpenCV (cv2.imshow), igual que ya hace
pare_detector.py. No usa navegador, no usa Flask, no usa http.server,
no necesita internet ni pip install: solo cv2/numpy/rclpy, que ya estan
instalados en el contenedor (los mismos que usa pare_detector.py).

Que hace:
  1. Se suscribe a /image_raw   -> panel de camara con mascara HSV
     (amarillo/blanco) superpuesta.
  2. Se suscribe a /color_detectado (publicado por pare_detector.py) -> arma
     el cuadro "Color Detectado: ...".
  3. Se suscribe a /cmd_vel (publicado por maze_solver.py) -> comportamiento
     (recto / girando / detenido) y velocidad media.
  4. Se suscribe a /odom_raw -> trayectoria (x, y) para el mapa y conteo de
     vueltas (heuristica simple por distancia al punto de partida).
  5. Se suscribe a /scan -> SOLO para estimar, de forma independiente, un
     error lateral e(t) analogo al que usa internamente maze_solver.py
     (no lee nada de ese proceso, lo recalcula con los mismos parametros
     de calibracion, solo para poder graficarlo).
  6. Dibuja todo (camara + trayectoria + grafico + barra de estado) en un
     solo lienzo con cv2 y lo muestra con cv2.imshow en una ventana.

Como correrlo (con /image_raw, /cmd_vel, /odom_raw y /color_detectado ya
activos, y con DISPLAY apuntando a una pantalla real, igual que
pare_detector.py):
    export DISPLAY=:0
    python3 dashboard_node.py
Cerrar con la tecla "q" o Ctrl+C.
"""

import math
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
from std_msgs.msg import String
from cv_bridge import CvBridge

# =======================================================================
# CALIBRACION (debe reflejar la de maze_solver.py SOLO para poder estimar
# e(t) de forma independiente; no se importa ni se altera ese archivo)
# =======================================================================
FRONT_OFFSET   = 180
SECTOR_HALF    = 12
FOLLOW_SIDE    = "left"
TARGET_DIST    = 0.18
PARE_ROJO_SEGUNDOS = 3.0
TOTAL_VUELTAS  = 3

TOPIC_IMAGEN   = "/image_raw"
TOPIC_COLOR    = "/color_detectado"
TOPIC_CMDVEL   = "/cmd_vel"
TOPIC_ODOM     = "/odom_raw"
TOPIC_SCAN     = "/scan"

WINDOW_NAME    = "Gran Prix CapyTown - Semana 16 - RC-2"
CANVAS_W, CANVAS_H = 1280, 760
FPS_RENDER     = 15

# Rangos HSV solo para PINTAR el carril en el panel de camara (informativo)
HSV_AMARILLO = (np.array([18, 80, 90]),  np.array([35, 255, 255]))
HSV_BLANCO   = (np.array([0, 0, 200]),   np.array([180, 40, 255]))

# Colores BGR (estetica "pit wall": negro carbono + rojo/verde/ambar carrera)
C_BG        = (17, 14, 11)
C_PANEL     = (27, 22, 18)
C_LINE      = (50, 42, 35)
C_TEXT      = (238, 241, 238)
C_MUTED     = (163, 148, 138)
C_RED       = (63, 67, 232)
C_GREEN     = (153, 199, 40)
C_AMBER     = (0, 163, 244)
C_YELLOW    = (63, 210, 255)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def put_text(img, text, org, scale=0.6, color=C_TEXT, thickness=1, font=cv2.FONT_HERSHEY_SIMPLEX):
    cv2.putText(img, text, org, font, scale, color, thickness, cv2.LINE_AA)


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_node")
        self.bridge = CvBridge()

        # --- estado de datos (todo se actualiza desde los callbacks) ---
        self.cam_frame = None
        self.color = "NINGUNO"
        self.color_edge_ts = None
        self.meta_reached = False

        self.lin = 0.0
        self.ang = 0.0
        self.speed_samples = deque(maxlen=400)

        self.traj = deque(maxlen=4000)
        self.start_xy = None
        self.away_from_start = False
        self.laps = 0

        self.lane_error_series = deque(maxlen=300)
        self.t0 = time.time()

        self.confetti = []

        self.create_subscription(Image, TOPIC_IMAGEN, self.on_image, 10)
        self.create_subscription(String, TOPIC_COLOR, self.on_color, 10)
        self.create_subscription(Twist, TOPIC_CMDVEL, self.on_cmdvel, 10)
        self.create_subscription(Odometry, TOPIC_ODOM, self.on_odom, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self.on_scan, 10)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, CANVAS_W, CANVAS_H)

        self.get_logger().info("dashboard_node iniciado. Ventana local abierta (tecla 'q' para salir).")

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
            if nuevo == "VERDE":
                self.meta_reached = True
                self.spawn_confetti()
        self.color = nuevo

    def on_cmdvel(self, msg):
        self.lin = msg.linear.x
        self.ang = msg.angular.z
        self.speed_samples.append(abs(msg.linear.x))

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.traj.append((x, y))
        if self.start_xy is None:
            self.start_xy = (x, y)
        d0 = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
        if d0 > 1.0:
            self.away_from_start = True
        elif d0 < 0.3 and self.away_from_start and self.laps < TOTAL_VUELTAS:
            self.laps += 1
            self.away_from_start = False

    def on_scan(self, scan):
        sign = 1.0 if FOLLOW_SIDE == "right" else -1.0
        deg = -90 if FOLLOW_SIDE == "right" else 90
        n = len(scan.ranges)
        vals = []
        for d in range(-SECTOR_HALF, SECTOR_HALF + 1):
            raw_deg = deg + FRONT_OFFSET + d
            ang = math.radians(raw_deg)
            idx = int(round((ang - scan.angle_min) / scan.angle_increment)) % n
            r = scan.ranges[idx]
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r <= scan.range_min or r > scan.range_max:
                continue
            vals.append(r)
        if not vals:
            return
        b = min(vals)
        error = sign * (b - TARGET_DIST)
        self.lane_error_series.append((time.time() - self.t0, error))

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

    # --------------------------- Render helpers ---------------------------
    def draw_panel_box(self, img, x, y, w, h, title):
        cv2.rectangle(img, (x, y), (x + w, y + h), C_PANEL, -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), C_LINE, 1)
        put_text(img, title, (x + 12, y + 22), 0.55, C_TEXT, 1)
        return (x + 10, y + 34, w - 20, h - 44)  # area util interior

    def draw_trajectory(self, img, x, y, w, h):
        cv2.rectangle(img, (x, y), (x + w, y + h), (13, 11, 9), -1)
        for gx in range(x, x + w, 20):
            cv2.line(img, (gx, y), (gx, y + h), C_LINE, 1)
        for gy in range(y, y + h, 20):
            cv2.line(img, (x, gy), (x + w, gy), C_LINE, 1)

        pts = list(self.traj)
        if len(pts) < 2:
            put_text(img, "Esperando /odom_raw...", (x + 10, y + h // 2), 0.5, C_MUTED, 1)
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        pad = 14
        sx = (w - 2 * pad) / (maxx - minx) if maxx - minx > 0.01 else 1.0
        sy = (h - 2 * pad) / (maxy - miny) if maxy - miny > 0.01 else 1.0
        s = min(sx, sy)

        def to_px(px, py):
            return (int(x + pad + (px - minx) * s), int(y + h - pad - (py - miny) * s))

        for i in range(1, len(pts)):
            cv2.line(img, to_px(*pts[i - 1]), to_px(*pts[i]), C_YELLOW, 2)
        last = to_px(*pts[-1])
        cv2.circle(img, last, 5, C_RED, -1)
        put_text(img, "e(t)", (last[0] + 8, last[1] - 6), 0.5, C_TEXT, 1)

    def draw_error_plot(self, img, x, y, w, h):
        cv2.rectangle(img, (x, y), (x + w, y + h), (13, 11, 9), -1)
        cv2.line(img, (x, y + h // 2), (x + w, y + h // 2), C_LINE, 1)
        series = list(self.lane_error_series)
        if len(series) < 2:
            put_text(img, "Esperando /scan...", (x + 10, y + h // 2 - 10), 0.5, C_MUTED, 1)
            return
        ts = [p[0] for p in series]
        es = [p[1] for p in series]
        mint, maxt = min(ts), max(ts)
        maxabs = max(0.05, max(abs(e) for e in es))
        pad = 8

        def to_px(t, e):
            px = x + pad + ((t - mint) / (maxt - mint) if maxt > mint else 0) * (w - 2 * pad)
            py = y + h // 2 - (e / maxabs) * (h // 2 - pad)
            return (int(px), int(py))

        for i in range(1, len(series)):
            cv2.line(img, to_px(*series[i - 1]), to_px(*series[i]), C_GREEN, 2)

    def render(self):
        img = np.full((CANVAS_H, CANVAS_W, 3), C_BG, dtype=np.uint8)

        # --- encabezado ---
        put_text(img, "GRAN PRIX CAPYTOWN - SEMANA 11 - RC-2", (24, 30), 0.75, C_TEXT, 2)
        put_text(img, 'Yahboom Pi5 Robocar - Escenario A "El Tambo" - Equipo Jose Anyelo',
                  (24, 52), 0.45, C_MUTED, 1)
        cv2.line(img, (0, 66), (CANVAS_W, 66), C_LINE, 1)

        pad = 16
        top = 78
        bottom_bar_h = 40
        panel_h = CANVAS_H - top - bottom_bar_h - pad
        panel_w = (CANVAS_W - 3 * pad) // 2

        # --- panel 1: camara ---
        x1, y1 = pad, top
        ix, iy, iw, ih = self.draw_panel_box(
            img, x1, y1, panel_w, panel_h,
            "1. Yahboom Pi5 Camera - Lane Segmentation (HSV & IPM)")
        if self.cam_frame is not None:
            resized = cv2.resize(self.cam_frame, (iw, ih))
            img[iy:iy + ih, ix:ix + iw] = resized
        else:
            cv2.rectangle(img, (ix, iy), (ix + iw, iy + ih), (8, 8, 8), -1)
            put_text(img, "Esperando /image_raw...", (ix + 14, iy + ih // 2), 0.55, C_MUTED, 1)

        # cuadro "Color Detectado" flotando sobre la camara
        if self.color == "ROJO":
            restante = max(0.0, PARE_ROJO_SEGUNDOS - (time.time() - (self.color_edge_ts or time.time())))
            txt = f"Color Detectado: Rojo - Deteniendose {restante:.1f}s"
            box_color = C_RED
        elif self.color == "VERDE":
            txt = "Color Detectado: Verde - Meta detectada!"
            box_color = C_GREEN
        else:
            txt = "Color Detectado: Ninguno - Via libre"
            box_color = C_MUTED
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        bx = ix + (iw - tw) // 2 - 14
        by = iy + 14
        cv2.rectangle(img, (bx, by), (bx + tw + 28, by + th + 24), (10, 10, 10), -1)
        cv2.rectangle(img, (bx, by), (bx + tw + 28, by + th + 24), box_color, 2)
        put_text(img, txt, (bx + 14, by + th + 8), 0.6, C_TEXT, 2)

        # --- panel 2: trayectoria + error ---
        x2, y2 = pad * 2 + panel_w, top
        ix2, iy2, iw2, ih2 = self.draw_panel_box(
            img, x2, y2, panel_w, panel_h,
            "2. Robot Trajectory & /Lane_Error Plot (Metros)")

        if abs(self.ang) < 0.05 and abs(self.lin) > 0.01:
            comportamiento = "Avanzando en linea recta"
        elif abs(self.lin) < 0.01 and abs(self.ang) < 0.01:
            comportamiento = "Detenido"
        elif self.ang > 0:
            comportamiento = "Girando a la izquierda"
        else:
            comportamiento = "Girando a la derecha"

        badge_txt = f"Comportamiento: {comportamiento}"
        (bw, bh), _ = cv2.getTextSize(badge_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (ix2, iy2), (ix2 + bw + 20, iy2 + bh + 16), (20, 17, 14), -1)
        cv2.rectangle(img, (ix2, iy2), (ix2 + bw + 20, iy2 + bh + 16), C_LINE, 1)
        put_text(img, badge_txt, (ix2 + 10, iy2 + bh + 6), 0.5, C_YELLOW, 1)

        split_y = iy2 + bh + 30
        split_h = ih2 - (bh + 30) - 4
        sub_w = (iw2 - 10) // 2
        self.draw_trajectory(img, ix2, split_y, sub_w, split_h)
        self.draw_error_plot(img, ix2 + sub_w + 10, split_y, sub_w, split_h)
        put_text(img, 'Trayectoria - "El Tambo"', (ix2, split_y - 6), 0.42, C_MUTED, 1)
        put_text(img, "/Lane_Error vs. Tiempo", (ix2 + sub_w + 10, split_y - 6), 0.42, C_MUTED, 1)

        # overlay de META (con confeti) sobre el panel derecho
        if self.meta_reached:
            overlay = img.copy()
            cv2.rectangle(overlay, (x2, y2), (x2 + panel_w, y2 + panel_h), (5, 5, 5), -1)
            img = cv2.addWeighted(overlay, 0.7, img, 0.3, 0)
            self.step_confetti(panel_w, panel_h, img[y2:y2 + panel_h, x2:x2 + panel_w])
            msg1 = "LLEGAMOS A LA META CAPYTOWN"
            msg2 = "SEMANA 11 - RC-2 !"
            (w1, h1), _ = cv2.getTextSize(msg1, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            (w2, h2), _ = cv2.getTextSize(msg2, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            cx = x2 + panel_w // 2
            cy = y2 + panel_h // 2
            put_text(img, msg1, (cx - w1 // 2, cy - 8), 0.9, C_AMBER, 2)
            put_text(img, msg2, (cx - w2 // 2, cy + h2 + 10), 0.9, C_AMBER, 2)

        # --- barra de estado inferior ---
        by0 = CANVAS_H - bottom_bar_h
        cv2.rectangle(img, (0, by0), (CANVAS_W, CANVAS_H), C_PANEL, -1)
        cv2.line(img, (0, by0), (CANVAS_W, by0), C_LINE, 1)
        vel_media = (sum(self.speed_samples) / len(self.speed_samples)) if self.speed_samples else 0.0
        status = (f"Robot Status: Autonomous Mode   |   "
                  f"Completed Vueltas: {self.laps}/{TOTAL_VUELTAS}   |   "
                  f"Velocidad Media: {vel_media:.2f} m/s   |   "
                  f"/cmd_vel: lin={self.lin:.2f} ang={self.ang:.2f}   |   "
                  f"Estado HSV: Amarillo/Blanco activo")
        put_text(img, status, (16, by0 + 26), 0.48, C_TEXT, 1, cv2.FONT_HERSHEY_DUPLEX)

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
            key = cv2.waitKey(max(1, int(period * 1000)))
            if key & 0xFF in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
