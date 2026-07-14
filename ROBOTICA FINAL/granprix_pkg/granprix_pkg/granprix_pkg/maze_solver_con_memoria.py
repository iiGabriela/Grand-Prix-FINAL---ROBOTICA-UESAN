#!/usr/bin/env python3
"""
maze_solver_con_memoria.py - Gran Prix CapyTown + Recordatorio de Callejones
=====================================================================
Une: seguir pared (control de 2 rayos, mantiene PARALELO) + detectar
situaciones + GIRAR con /odom_raw + MEMORIA de callejones entre rondas.

NOVEDAD:
  - Ronda 1: explora y guarda que celdas fueron callejones (entrada=salida).
  - Ronda 2: evita esas celdas conocidas (toma ruta mas corta).

Regla de la mano configurable con UN solo parametro: FOLLOW_SIDE.
  "right" -> mano derecha
  "left"  -> mano izquierda

=== FIXES aplicados sobre la version original de con_memoria ===
1) La formula que calculaba la "celda vecina" (sig_celda) tenia
   IZQUIERDA y DERECHA invertidas -> nunca comparaba la celda correcta.
   Se centralizo en un solo metodo: _celda_lado_seguido().
2) En ronda 1 se guardaba pos_to_cell(x, y) (celda ACTUAL) como
   callejon, pero en ronda 2 se comparaba contra una celda "un paso
   adelante" (offset). Ahora ambas rondas usan el MISMO calculo
   (_celda_lado_seguido), asi que apuntan al mismo punto del mapa.
3) Se restauro el cooldown de color (ignorar_color_hasta) para evitar
   que el PARE se detecte varias veces seguidas mientras el robot gira.
"""

import math
import rclpy
import json
import os
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ===================== CALIBRACION =====================
FRONT_OFFSET  = 180
SECTOR_HALF   = 12
FRONT_HALF    = 7
BLIND_ZONES   = [(15, 30), (-30, -15)]

FOLLOW_SIDE   = "left"     # "left" o "right"

# Seguir pared (control de 2 rayos: distancia + angulo)
TARGET_DIST   = 0.18
KP            = 1.0
KD            = 1.5
DIAG_OFFSET   = 40
LOOKAHEAD     = 0.55
LINEAR_SPEED  = 0.10
MAX_ANGULAR   = 0.80

# Deteccion de situaciones
OPEN_THRESH   = 0.50
FRONT_BLOCK   = 0.22
SIDE_EMPTY    = 0.08

# Giros y maniobras
TURN_SPEED    = 0.6
TURN_TOL      = 0.05
CREEP_DIST    = 0.28
CREEP_SPEED   = 0.08
GRACE_DIST    = 0.35
CONFIRM_TICKS = 3

# Meta (salida a espacio abierto)
ENABLE_META   = True
META_OPEN     = 1.3
META_FRAMES   = 15

# Comunicacion con el nodo de camara
TOPIC_COLOR              = "/color_detectado"
PARE_ROJO_SEGUNDOS       = 3.0
COOLDOWN_COLOR_SEGUNDOS  = 1.5   # (FIX 3) restaurado: evita relecturas del mismo PARE al girar

# Calibracion inicial
CALIB_TICKS      = 20
CALIB_MAX_TICKS  = 100

# ===== MEMORIA DE CALLEJONES =====
CELL_SIZE        = 0.60   # m: tamano de celda de 60cm (como la pista)
# Se guarda al lado del propio script (ruta que SIEMPRE existe, sin adivinar
# workspaces ni usuarios). Si necesitas otra ubicacion, pon una ruta absoluta
# aqui en vez de la linea de abajo.
CALLEJONES_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "callejones_ronda1.json")
RONDA            = 1      # cambiar a 1 para explorar y guardar memoria; 2 para usarla
CELL_TOLERANCE   = 1      # celdas de margen por error de odometria
# ==========================================


def norm180(deg):
    return (deg + 180) % 360 - 180


def in_blind(raw_deg):
    a = norm180(raw_deg)
    for lo, hi in BLIND_ZONES:
        if lo <= a <= hi:
            return True
    return False


def ang_diff(target, current):
    d = target - current
    return math.atan2(math.sin(d), math.cos(d))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pos_to_cell(x, y):
    """Convierte posicion (x,y) en indice de celda (cx, cy)."""
    cx = int(round(x / CELL_SIZE))
    cy = int(round(y / CELL_SIZE))
    return (cx, cy)


def celda_dentro_margen(celda_a, celda_b, tol=CELL_TOLERANCE):
    """Compara dos celdas con tolerancia (odometria imperfecta)."""
    cx_a, cy_a = celda_a
    cx_b, cy_b = celda_b
    return abs(cx_a - cx_b) <= tol and abs(cy_a - cy_b) <= tol


class MazeSolver(Node):
    def __init__(self):
        super().__init__("maze_solver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom_raw", self.on_odom, 10)
        self.create_subscription(String, TOPIC_COLOR, self.on_color, 10)

        self.color_actual = "NINGUNO"
        self.color_anterior_edge = None
        self.pare_inicio = None
        self.estado_antes_pare = None
        self.ignorar_color_hasta = 0.0   # (FIX 3)

        self.front = self.left = self.right = None
        self.side_perp = self.side_diag = None
        self.x = self.y = self.yaw = None

        self.turn_target = None
        self.creep_ref = None
        self.next_delta = 0.0
        self.grace_ref = None

        self.prev_error = None
        self.prev_time = None

        self.confirm = 0
        self.confirm_action = None
        self.meta_count = 0

        self.follow_right = (FOLLOW_SIDE == "right")

        # ===== MEMORIA DE CALLEJONES =====
        self.ronda = RONDA
        self.celdas_visitadas = {}
        self.entrada_callejon = None
        self.callejones_conocidos = set()
        self.callejones_evitar = set()
        self._callejones_guardados = False

        if self.ronda == 2:
            self._cargar_callejones()
        # ==========================================

        self.state = "FOLLOW"  # arranca directo (sin calibracion)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"maze_solver iniciado. Ronda {self.ronda}, Mano {FOLLOW_SIDE.upper()}. "
            f"Archivo de memoria: {CALLEJONES_FILE}. Ctrl+C para parar.")

    # ===== MEMORIA DE CALLEJONES =====
    def _cargar_callejones(self):
        try:
            with open(CALLEJONES_FILE, "r") as f:
                data = json.load(f)
                self.callejones_evitar = set(tuple(c) for c in data["callejones"])
                self.get_logger().info(f"Callejones cargados para evitar: {self.callejones_evitar}")
        except FileNotFoundError:
            self.get_logger().warn(f"No se encontro {CALLEJONES_FILE}. Ronda 2 sin memoria.")
        except Exception as e:
            self.get_logger().error(f"Error cargando callejones: {e}")

    def _guardar_callejones(self):
        try:
            with open(CALLEJONES_FILE, "w") as f:
                json.dump({"callejones": [list(c) for c in self.callejones_conocidos]}, f)
                self.get_logger().info(f"Callejones guardados: {self.callejones_conocidos}")
        except Exception as e:
            self.get_logger().error(f"Error guardando callejones: {e}")

    def _celda_es_callejon(self, celda):
        for cal in self.callejones_evitar:
            if celda_dentro_margen(celda, cal, CELL_TOLERANCE):
                return True
        return False

    def _celda_lado_seguido(self):
        """(FIX 1 y 2) Celda contigua hacia el lado que se esta siguiendo
        (el lado 'near'). Se usa TANTO en ronda 1 (para registrar el
        callejon) COMO en ronda 2 (para verificar si hay que evitarlo),
        asi ambas rondas apuntan siempre al mismo punto del mapa.

        Vector derecha del robot = (sin(yaw), -cos(yaw))
        Vector izquierda del robot = (-sin(yaw), cos(yaw))
        """
        if self.follow_right:
            sig_x = self.x + CELL_SIZE * math.sin(self.yaw)
            sig_y = self.y - CELL_SIZE * math.cos(self.yaw)
        else:
            sig_x = self.x - CELL_SIZE * math.sin(self.yaw)
            sig_y = self.y + CELL_SIZE * math.cos(self.yaw)
        return pos_to_cell(sig_x, sig_y)
    # ==========================================

    def sector_distance(self, scan, center_deg, on_empty=None, half=SECTOR_HALF):
        n = len(scan.ranges)
        vals = []
        for d in range(-half, half + 1):
            raw_deg = center_deg + FRONT_OFFSET + d
            if in_blind(raw_deg):
                continue
            ang = math.radians(raw_deg)
            idx = int(round((ang - scan.angle_min) / scan.angle_increment)) % n
            r = scan.ranges[idx]
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r <= scan.range_min or r > scan.range_max:
                continue
            vals.append(r)
        if not vals:
            return on_empty if on_empty is not None else scan.range_max
        return min(vals)

    def on_color(self, msg):
        self.color_actual = msg.data

    def on_scan(self, scan):
        self.front = self.sector_distance(scan, 0, half=FRONT_HALF)
        self.left  = self.sector_distance(scan, 90,  on_empty=SIDE_EMPTY)
        self.right = self.sector_distance(scan, -90, on_empty=SIDE_EMPTY)

        if self.follow_right:
            perp, diag = -90, -90 + DIAG_OFFSET
        else:
            perp, diag = 90, 90 - DIAG_OFFSET
        self.side_perp = self.sector_distance(scan, perp, on_empty=SIDE_EMPTY)
        self.side_diag = self.sector_distance(scan, diag)

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def dist_from(self, ref):
        return math.hypot(self.x - ref[0], self.y - ref[1])

    def publish(self, lin, ang):
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        try:
            self.pub.publish(cmd)
        except Exception:
            pass

    def stop(self):
        self.publish(0.0, 0.0)

    def pd_follow(self):
        sign = 1.0 if self.follow_right else -1.0
        b = self.side_perp
        a = self.side_diag
        theta = math.radians(DIAG_OFFSET)

        if 0.02 < a < 1.0 and 0.02 < b < 1.0:
            alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
            dist = b * math.cos(alpha)
            future = dist + LOOKAHEAD * math.sin(alpha)
            error = future - TARGET_DIST
        else:
            error = b - TARGET_DIST

        now = self.get_clock().now().nanoseconds * 1e-9
        deriv = 0.0
        if self.prev_error is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 1e-3:
                deriv = (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time = now

        ang = clamp(-sign * (KP * error + KD * deriv), -MAX_ANGULAR, MAX_ANGULAR)
        lin = LINEAR_SPEED * (1.0 - 0.5 * abs(ang) / MAX_ANGULAR)
        return lin, ang

    def reset_pd(self):
        self.prev_error = None
        self.prev_time = None

    def start_turn(self, delta):
        self.turn_target = self.yaw + delta
        self.state = "TURN"

    def tick(self):
        if self.front is None or self.yaw is None:
            return

        # ============ FIN (verde detectado: se acabo el recorrido) ============
        if self.state == "FIN":
            self.stop()
            if self.ronda == 1 and not self._callejones_guardados:
                self._guardar_callejones()
                self._callejones_guardados = True
            return

        # ============ PARE (rojo detectado: pausa de 3 segundos) ============
        if self.state == "PARE":
            ahora = self.get_clock().now().nanoseconds * 1e-9
            if ahora - self.pare_inicio >= PARE_ROJO_SEGUNDOS:
                self.state = self.estado_antes_pare or "FOLLOW"
                self.reset_pd()
                # (FIX 3) suma, no multiplicacion
                self.ignorar_color_hasta = ahora + COOLDOWN_COLOR_SEGUNDOS
                self.get_logger().info(f"[PARE] {PARE_ROJO_SEGUNDOS}s cumplidos, continuando.")
            else:
                self.stop()
                return

        # ============ Reaccion al color (flanco: solo al CAMBIAR) ============
        ahora = self.get_clock().now().nanoseconds * 1e-9
        if ahora >= self.ignorar_color_hasta and self.color_actual != self.color_anterior_edge:
            nuevo = self.color_actual
            self.color_anterior_edge = nuevo

            if nuevo == "VERDE":
                self.get_logger().info("[CAMARA] VERDE detectado -> fin de trayecto.")
                self.stop()
                self.state = "FIN"
                return

            if nuevo == "ROJO" and self.state != "PARE":
                self.get_logger().info("[CAMARA] ROJO detectado -> pausa 3s.")
                self.estado_antes_pare = self.state
                self.pare_inicio = self.get_clock().now().nanoseconds * 1e-9
                self.stop()
                self.state = "PARE"
                return

        f, l, r = self.front, self.left, self.right

        if ENABLE_META and self.state == "FOLLOW":
            if f > META_OPEN and l > META_OPEN and r > META_OPEN:
                self.meta_count += 1
                if self.meta_count >= META_FRAMES:
                    self.state = "META"
            else:
                self.meta_count = 0

        # ===== REGISTRO DE CELDAS =====
        celda_actual = pos_to_cell(self.x, self.y)
        if celda_actual not in self.celdas_visitadas:
            self.celdas_visitadas[celda_actual] = self.get_clock().now().nanoseconds * 1e-9
        # ==================================================================

        # ============ FOLLOW ============
        if self.state == "FOLLOW":
            past_grace = (self.grace_ref is None or
                          self.dist_from(self.grace_ref) > GRACE_DIST)

            near = r if self.follow_right else l
            far  = l if self.follow_right else r
            near_open = near >= OPEN_THRESH
            far_open  = far  >= OPEN_THRESH
            front_blocked = f < FRONT_BLOCK

            action = None
            if near_open and past_grace:
                action = "NEAR"
            elif front_blocked:
                action = "FAR" if far_open else "UTURN"

            # ===== RONDA 2: EVITAR CALLEJONES CONOCIDOS =====
            if self.ronda == 2 and action == "NEAR":
                sig_celda = self._celda_lado_seguido()   # (FIX 1/2) mismo calculo que ronda 1
                if self._celda_es_callejon(sig_celda):
                    self.get_logger().info(
                        f"[FOLLOW] Evitando callejon conocido en celda {sig_celda}. "
                        f"Continuando recto.")
                    action = None  # NO girar, continuar recto
            # =========================================================

            if action is not None and action == self.confirm_action:
                self.confirm += 1
            else:
                self.confirm_action = action
                self.confirm = 1 if action is not None else 0

            if action is not None and self.confirm >= CONFIRM_TICKS:
                self.get_logger().info(f"[FOLLOW] {action} "
                                       f"(F={f:.2f} L={l:.2f} R={r:.2f})")
                self.confirm = 0
                if action == "NEAR":
                    self.creep_ref = (self.x, self.y)
                    self.next_delta = -math.pi/2 if self.follow_right else math.pi/2
                    self.state = "CREEP"
                    # ===== RONDA 1: registra la celda a la que ENTRA =====
                    if self.ronda == 1:
                        self.entrada_callejon = self._celda_lado_seguido()  # (FIX 1/2)
                    # ==============================================
                elif action == "FAR":
                    self.reset_pd()
                    self.start_turn(math.pi/2 if self.follow_right else -math.pi/2)
                elif action == "UTURN":
                    self.reset_pd()
                    self.start_turn(math.pi)
                    # ===== RONDA 1: confirma que esa celda es callejon =====
                    if self.ronda == 1 and self.entrada_callejon is not None:
                        self.callejones_conocidos.add(self.entrada_callejon)
                        self.get_logger().info(
                            f"[CALLEJON] Detectado callejon en {self.entrada_callejon}")
                        self.entrada_callejon = None
                    # ===============================================
                return

            lin, ang = self.pd_follow()
            self.publish(lin, ang)
            self.get_logger().info(
                f"[FOLLOW] F={f:4.2f} L={l:4.2f} R={r:4.2f} v={lin:4.2f} w={ang:+4.2f}",
                throttle_duration_sec=1.0)
            return

        # ============ CREEP ============
        if self.state == "CREEP":
            if self.dist_from(self.creep_ref) < CREEP_DIST:
                self.publish(CREEP_SPEED, 0.0)
            else:
                self.reset_pd()
                self.start_turn(self.next_delta)
            return

        # ============ TURN ============
        if self.state == "TURN":
            rem = ang_diff(self.turn_target, self.yaw)
            if abs(rem) < TURN_TOL:
                self.stop()
                self.reset_pd()
                self.grace_ref = (self.x, self.y)
                self.state = "FOLLOW"
                self.get_logger().info("[TURN] completado.")
            else:
                speed = clamp(abs(rem) * 1.5, 0.15, TURN_SPEED)
                self.publish(0.0, speed * (1.0 if rem > 0 else -1.0))
            return

        # ============ META ============
        if self.state == "META":
            self.stop()
            self.get_logger().info("*** META alcanzada. Robot detenido. ***",
                                   throttle_duration_sec=2.0)
            return


def main():
    rclpy.init()
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
