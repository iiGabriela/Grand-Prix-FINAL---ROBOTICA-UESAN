#!/usr/bin/env python3
"""
maze_solver.py - Gran Prix CapyTown - FSM de navegacion (sin camara/PARE aun)
Une: seguir pared (control de 2 rayos, mantiene PARALELO) + detectar
situaciones + GIRAR con /odom_raw.

Regla de la mano configurable con UN solo parametro: FOLLOW_SIDE.
  "right" -> mano derecha: prioridad derecha > frente > izquierda > media vuelta
  "left"  -> mano izquierda: prioridad izquierda > frente > derecha > media vuelta
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ===================== CALIBRACION =====================
FRONT_OFFSET  = 180
SECTOR_HALF   = 12         # ancho de los sectores LATERALES
FRONT_HALF    = 7          # ancho del sector FRONTAL (angosto: evita captar
                           # la pared de al lado si el robot va un poco torcido)
BLIND_ZONES   = [(15, 30), (-30, -15)]

FOLLOW_SIDE   = "left"     # "left" o "right"  <- unico cambio para probar cada mano

# Seguir pared (control de 2 rayos: distancia + angulo)
TARGET_DIST   = 0.18       # m: distancia deseada a la pared seguida
KP            = 1.0        # ganancia proporcional
KD            = 1.5        # ganancia derivativa (amortigua)
DIAG_OFFSET   = 40         # grados hacia adelante del rayo diagonal (para el angulo)
LOOKAHEAD     = 0.55       # m: cuanto "mira adelante" para anticipar la correccion
LINEAR_SPEED  = 0.10
MAX_ANGULAR   = 0.80

# Deteccion de situaciones
OPEN_THRESH   = 0.50       # lateral mayor a esto = apertura
FRONT_BLOCK   = 0.22       # frente menor a esto = pared al frente
SIDE_EMPTY    = 0.08       # failsafe: lado sin lecturas (pared en zona ciega) = muy cerca

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

# Comunicacion con el nodo de camara (pare_detector.py)
TOPIC_COLOR         = "/color_detectado"
PARE_ROJO_SEGUNDOS  = 3.0

# Calibracion inicial (corrige el sensor/laser si esta chueco/desalineado)
# Al arrancar, el robot se queda quieto y promedia el angulo a la pared
# (mismo calculo de 2 rayos que pd_follow) para usarlo como offset fijo.
# IMPORTANTE: colocar el robot alineado/paralelo a la pared antes de arrancar.
CALIB_TICKS      = 20   # muestras validas a promediar (~1s a 20Hz)
CALIB_MAX_TICKS  = 100  # timeout de espera (~5s a 20Hz) si no hay pared visible
# =======================================================


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


class MazeSolver(Node):
    def __init__(self):
        super().__init__("maze_solver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom_raw", self.on_odom, 10)
        self.create_subscription(String, TOPIC_COLOR, self.on_color, 10)

        # --- Estado para reaccionar al color (ROJO = pausa 3s, VERDE = fin) ---
        self.color_actual = "NINGUNO"
        self.color_anterior_edge = None   # para detectar el CAMBIO de color (flanco)
        self.pare_inicio = None           # timestamp en que empezo la pausa por rojo
        self.estado_antes_pare = None     # a que estado volver despues de la pausa

        self.front = self.left = self.right = None
        self.side_perp = self.side_diag = None
        self.x = self.y = self.yaw = None

        self.state = "CALIBRATING"
        self.turn_target = None
        self.creep_ref = None
        self.next_delta = 0.0
        self.grace_ref = None

        # --- Calibracion inicial (offset del angulo a la pared) ---
        self.calib_offset = 0.0
        self.calib_samples = []
        self.calib_ticks = 0

        self.prev_error = None
        self.prev_time = None

        self.confirm = 0
        self.confirm_action = None
        self.meta_count = 0

        self.follow_right = (FOLLOW_SIDE == "right")
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"maze_solver iniciado. Mano {FOLLOW_SIDE.upper()}. "
            f"Calibrando sensor (mantener el robot quieto y alineado a la pared)... "
            f"Ctrl+C para parar.")

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
        """Recibe el color detectado por el nodo pare_detector.py (otro proceso)."""
        self.color_actual = msg.data  # "ROJO", "VERDE" o "NINGUNO"

    def on_scan(self, scan):
        # frente ANGOSTO; lados anchos
        self.front = self.sector_distance(scan, 0, half=FRONT_HALF)
        self.left  = self.sector_distance(scan, 90,  on_empty=SIDE_EMPTY)
        self.right = self.sector_distance(scan, -90, on_empty=SIDE_EMPTY)

        # rayos para el control de angulo (perpendicular + diagonal al frente)
        if self.follow_right:
            perp, diag = -90, -90 + DIAG_OFFSET
        else:
            perp, diag = 90, 90 - DIAG_OFFSET
        self.side_perp = self.sector_distance(scan, perp, on_empty=SIDE_EMPTY)
        self.side_diag = self.sector_distance(scan, diag)   # vacio -> lejos -> fallback

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

    def compute_alpha(self):
        """Angulo crudo (sin calibrar) hacia la pared, con los 2 rayos.
        Devuelve None si no hay pared clara en ambos rayos."""
        b = self.side_perp      # perpendicular a la pared
        a = self.side_diag      # diagonal hacia el frente
        theta = math.radians(DIAG_OFFSET)
        if a is not None and b is not None and 0.02 < a < 1.0 and 0.02 < b < 1.0:
            return math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        return None

    def pd_follow(self):
        """Control de 2 rayos: corrige distancia Y angulo -> va PARALELO."""
        sign = 1.0 if self.follow_right else -1.0
        b = self.side_perp      # perpendicular a la pared
        raw_alpha = self.compute_alpha()

        # si hay pared clara en ambos rayos, usa el angulo (2 rayos, ya calibrado);
        # si no (apertura / sin pared), cae a control por distancia simple
        if raw_alpha is not None:
            alpha = raw_alpha - self.calib_offset   # corrige sensor chueco
            dist = b * math.cos(alpha)
            future = dist + LOOKAHEAD * math.sin(alpha)   # distancia proyectada adelante
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

        # ============ CALIBRATING (offset del sensor, antes de arrancar) ============
        if self.state == "CALIBRATING":
            self.stop()  # el robot se queda quieto mientras calibra
            self.calib_ticks += 1

            raw_alpha = self.compute_alpha()
            if raw_alpha is not None:
                self.calib_samples.append(raw_alpha)

            listo = len(self.calib_samples) >= CALIB_TICKS
            agoto_tiempo = self.calib_ticks >= CALIB_MAX_TICKS

            if listo or agoto_tiempo:
                if self.calib_samples:
                    self.calib_offset = sum(self.calib_samples) / len(self.calib_samples)
                    self.get_logger().info(
                        f"[CALIBRACION] offset = {math.degrees(self.calib_offset):.2f} deg "
                        f"({len(self.calib_samples)} muestras). Iniciando FOLLOW.")
                else:
                    self.calib_offset = 0.0
                    self.get_logger().info(
                        "[CALIBRACION] no se detecto pared cercana, offset=0. "
                        "Iniciando FOLLOW.")
                self.reset_pd()
                self.grace_ref = (self.x, self.y)
                self.state = "FOLLOW"
            return

        # ============ FIN (verde detectado: se acabo el recorrido) ============
        if self.state == "FIN":
            self.stop()
            return

        # ============ PARE (rojo detectado: pausa de 3 segundos) ============
        if self.state == "PARE":
            ahora = self.get_clock().now().nanoseconds * 1e-9
            if ahora - self.pare_inicio >= PARE_ROJO_SEGUNDOS:
                self.state = self.estado_antes_pare or "FOLLOW"
                self.reset_pd()
                self.get_logger().info(f"[PARE] {PARE_ROJO_SEGUNDOS}s cumplidos, continuando.")
            else:
                self.stop()
                return

        # ============ Reaccion al color (flanco: solo al CAMBIAR) ============
        if self.color_actual != self.color_anterior_edge:
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
                elif action == "FAR":
                    self.reset_pd()
                    self.start_turn(math.pi/2 if self.follow_right else -math.pi/2)
                elif action == "UTURN":
                    self.reset_pd()
                    self.start_turn(math.pi)
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
