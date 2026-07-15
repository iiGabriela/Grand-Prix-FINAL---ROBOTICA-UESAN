[README (1) (1).md](https://github.com/user-attachments/files/30066979/README.1.1.md)
## **Gran Prix CapyTown — El Qhapaq Ñan: El Laberinto del Chaski**

Proyecto Final de Robótica l 2026-I · ESAN

# **1. ESTRUCTURA DEL PROYECTO**

El proyecto está organizado como un paquete ROS2 Humble dentro de un contenedor Docker en una Raspberry Pi 5. Consta de 3 nodos principales que corren en paralelo:

**Carpeta del proyecto:** granprix_pkg/granprix_pkg/granprix_pkg/

**Archivo 1:** pare_detector.py (121 líneas) — Nodo de visión (cámara)

**Archivo 2:** maze_solver_con_memoria.py (492 líneas) — Nodo principal (cerebro del robot)

**Archivo 3:** dashboard_node.py (745 líneas) — Nodo de monitoreo (visualización)

**Archivo 4:** setup.py (27 líneas) — Configuración del paquete ROS2

# **2. PARE_DETECTOR.PY — EL OJO DEL ROBOT**

### **Ubicación y propósito**

**Archivo:** granprix_pkg/granprix_pkg/granprix_pkg/pare_detector.py

**Líneas:** 1 a 121

**Propósito:** Detectar colores ROJO (señal PARE) y VERDE (META) usando la cámara USB. No mueve el robot. Solo observa y publica qué color ve.

### **Sección 2.1: Importaciones y configuración (líneas 18-32)**

TOPIC_IMAGEN = "/image_raw"

TOPIC_COLOR = "/color_detectado"

MIN_AREA_COLOR = 600

Se definen los tópicos de entrada (imagen de la cámara) y salida (color detectado). MIN_AREA_COLOR = 600 significa que necesita al menos 600 píxeles de un color para considerarlo válido (evita ruido).

### **Sección 2.2: Rangos de color HSV (líneas 33-44)**

COLOR_RANGES = {

"ROJO": [

(np.array([0, 70, 40]), np.array([10, 255, 255])),

(np.array([165, 70, 40]), np.array([180, 255, 255])),

],

"VERDE": [

(np.array([36, 80, 60]), np.array([85, 255, 255])),

],

}

Define qué valores HSV (Hue, Saturation, Value) corresponden a ROJO y VERDE. El rojo tiene DOS rangos porque en HSV el rojo está en ambos extremos (0-10 y 165-180). Se usa HSV en vez de RGB porque es más resistente a cambios de iluminación.

### **Sección 2.3: Función detectar_color() (líneas 46-64)**

def detectar_color(frame_hsv):

mejor_color = "NINGUNO"

mejor_area = 0

for nombre, rangos in COLOR_RANGES.items():

mascara_total = ...

mascara_total = cv2.erode(mascara_total, None, iterations=2)

mascara_total = cv2.dilate(mascara_total, None, iterations=2)

area = cv2.countNonZero(mascara_total)

if area > mejor_area:

mejor_color = nombre

if mejor_area >= MIN_AREA_COLOR:

return mejor_color

return "NINGUNO"

Recibe un frame en HSV. Para cada color (ROJO, VERDE): crea una máscara (píxeles que caen en el rango), aplica erosión (quita ruido) y dilatación (rellena huecos), cuenta los píxeles. El color con más área gana. Si ninguno supera los 600 píxeles, devuelve "NINGUNO".

### **Sección 2.4: Clase ParedetectorNode (líneas 66-105)**

class ParedetectorNode(Node):

def __init__(self):

self.suscripcion = self.create_subscription(

Image, TOPIC_IMAGEN, self.callback_imagen, 10)

self.publicador = self.create_publisher(String, TOPIC_COLOR, 10)

Se suscribe a /image_raw (recibe cada frame de la cámara) y publica en /color_detectado (el resultado).

### **Sección 2.5: callback_imagen() (líneas 81-104)**

def callback_imagen(self, msg):

frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

color = detectar_color(frame_hsv)

salida = String()

salida.data = color

self.publicador.publish(salida)

Cada vez que llega un frame: convierte BGR a HSV, detecta el color, y publica el resultado ("ROJO", "VERDE" o "NINGUNO") en /color_detectado. También muestra una ventana de debug con cv2.imshow (líneas 101-102).

### **Sección 2.6: Función main() (líneas 105-121)**

def main(args=None):

rclpy.init(args=args)

nodo = ParedetectorNode()

rclpy.spin(nodo)

Inicia ROS2, crea el nodo, y lo mantiene corriendo hasta Ctrl+C.

# **3. MAZE_SOLVER_CON_MEMORIA.PY — EL CEREBRO DEL ROBOT**

### **Ubicación y propósito**

**Archivo:** granprix_pkg/granprix_pkg/granprix_pkg/maze_solver_con_memoria.py

**Líneas:** 1 a 492

**Propósito:** Controlar el robot autónomamente: seguir paredes, decidir giros, respetar PARE, llegar a META, y en Ronda 2 evitar callejones conocidos de Ronda 1.

### **Sección 3.1: Descripción y fixes (líneas 1-35)**

Documentación del archivo. Menciona 3 fixes aplicados: (1) fórmula de celda vecina corregida, (2) ambas rondas usan el mismo cálculo de celda, (3) cooldown de color para evitar detecciones repetidas del PARE al girar.

### **Sección 3.2: Importaciones (líneas 36-44)**

import math, rclpy, json, os

from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

from std_msgs.msg import String

Importa: math (trigonometría), json (guardar/leer callejones), os (rutas de archivo), y los tipos de mensajes ROS2 (LaserScan para LiDAR, Odometry para posición, Twist para mover motores, String para color).

### **Sección 3.3: Parámetros de calibración (líneas 47-99)**

FRONT_OFFSET = 180 \# LiDAR montado al revés (rotado 180°)

SECTOR_HALF = 12 \# ancho de sectores laterales (±12°)

FRONT_HALF = 7 \# ancho del sector frontal (±7°, angosto)

BLIND_ZONES = [(15, 30), (-30, -15)] \# auto-oclusión del robot

FRONT_OFFSET = 180: El LiDAR MS200 está montado al revés en este robot. Lo que el sensor llama "0°" es realmente la parte de atrás del robot. El offset de 180° corrige esto matemáticamente.

BLIND_ZONES: Zonas donde el LiDAR ve partes del propio robot (a ±20-25°). Se ignoran esos rayos para evitar falsos positivos.

FRONT_HALF = 7: El cono frontal es más angosto (±7° en vez de ±12°) para evitar que rayos del borde capten la pared lateral cuando el robot va ligeramente torcido.

### **Sección 3.4: Parámetro de estrategia (línea 52)**

FOLLOW_SIDE = "left" \# "left" o "right"

Define la regla de la mano. Con "left", el robot sigue la pared izquierda y prioriza girar a la izquierda en cada intersección. Este es el ÚNICO parámetro que controla toda la estrategia de navegación.

### **Sección 3.5: Parámetros de seguimiento de pared (líneas 55-62)**

TARGET_DIST = 0.18 \# mantener robot a 18 cm de la pared

KP = 1.0 \# ganancia proporcional

KD = 1.5 \# ganancia derivativa (amortigua)

DIAG_OFFSET = 40 \# ángulo del rayo diagonal (grados)

LOOKAHEAD = 0.55 \# cuánto "mira adelante" (metros)

LINEAR_SPEED = 0.10 \# velocidad de avance (m/s)

MAX_ANGULAR = 0.80 \# giro máximo (rad/s)

TARGET_DIST = 0.18: Se mantiene a 18 cm de la pared (no 10 cm, porque el LiDAR no mide por debajo de 12 cm). KP = fuerza de corrección. KD = amortiguación (evita zigzag). LOOKAHEAD = proyecta cuánto adelante para anticipar correcciones.

### **Sección 3.6: Parámetros de detección de situaciones (líneas 65-67)**

OPEN_THRESH = 0.50 \# distancia lateral > 50 cm = apertura

FRONT_BLOCK = 0.22 \# distancia frontal < 22 cm = pared

SIDE_EMPTY = 0.08 \# sin lecturas laterales = pared muy cerca

OPEN_THRESH: Si la distancia lateral supera 50 cm, la pared "desapareció" → hay una bifurcación. FRONT_BLOCK: Si hay algo a menos de 22 cm enfrente → pared bloqueando. SIDE_EMPTY: Failsafe: si el LiDAR no da lecturas de un lado (zona ciega), asume pared MUY cerca (0.08 m) en vez de espacio abierto.

### **Sección 3.7: Parámetros de giros (líneas 70-76)**

TURN_SPEED = 0.6 \# velocidad de giro (rad/s)

TURN_TOL = 0.05 \# tolerancia (±3° para considerar "llegué")

CREEP_DIST = 0.28 \# avance antes de girar a la izquierda (m)

GRACE_DIST = 0.35 \# no re-evaluar tras un giro en 35 cm

CONFIRM_TICKS = 3 \# confirmar decisión 3 ciclos seguidos

CREEP_DIST: Antes de girar a la izquierda, el robot avanza 28 cm para pasar la esquina sin chocar. GRACE_DIST: Después de girar, no vuelve a evaluar la misma dirección en 35 cm (anti-oscilación). CONFIRM_TICKS: Espera 3 lecturas consecutivas iguales antes de decidir (anti-ruido).

### **Sección 3.8: Parámetros de cámara y PARE (líneas 82-85)**

TOPIC_COLOR = "/color_detectado"

PARE_ROJO_SEGUNDOS = 3.0

COOLDOWN_COLOR_SEGUNDOS = 1.5

El robot escucha /color_detectado (publicado por pare_detector.py). Cuando ve ROJO, para 3 segundos. COOLDOWN evita que detecte el mismo cartel PARE dos veces seguidas al girar.

### **Sección 3.9: Parámetros de memoria de callejones (líneas 91-99)**

CELL_SIZE = 0.60 \# tamaño de celda (60 cm como la pista)

CALLEJONES_FILE = os.path.join(os.path.dirname(...), "callejones_ronda1.json")

RONDA = 1 \# 1 = explorar y guardar, 2 = usar memoria

CELL_TOLERANCE = 1 \# margen de ±1 celda por error de odometría

CELL_SIZE = 0.60: Divide la pista en celdas de 60 cm (como la rejilla real de la pista). RONDA: Se cambia manualmente a 1 o 2 antes de cada corrida. CELL_TOLERANCE: Acepta ±1 celda de error porque la odometría se degrada con los giros.

### **Sección 3.10: Funciones auxiliares (líneas 103-140)**

def norm180(deg): \# normaliza ángulo a (-180, 180]

def in_blind(raw_deg): \# verifica si un rayo está en zona ciega

def ang_diff(target, cur): \# diferencia angular más corta

def yaw_from_quat(q): \# convierte cuaternión a ángulo yaw

def clamp(v, lo, hi): \# limita un valor entre lo y hi

def pos_to_cell(x, y): \# convierte posición (x,y) a celda (cx,cy)

def celda_dentro_margen(): \# compara celdas con tolerancia

Funciones matemáticas usadas en todo el código. La más importante es pos_to_cell(): convierte coordenadas del odómetro (ej. x=1.8, y=3.0) a índice de celda (ej. celda 3, 5) dividiendo entre CELL_SIZE (0.60 m).

### **Sección 3.11: Constructor __init__ (líneas 143-191)**

class MazeSolver(Node):

def __init__(self):

self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

self.create_subscription(Odometry, "/odom_raw", self.on_odom, 10)

self.create_subscription(String, TOPIC_COLOR, self.on_color, 10)

Crea las conexiones ROS2: publica en /cmd_vel (mover motores), se suscribe a /scan (LiDAR), /odom_raw (posición), /color_detectado (cámara). Inicializa todas las variables de estado, carga callejones si es Ronda 2, y arranca en estado "FOLLOW".

### **Sección 3.12: Funciones de memoria de callejones (líneas 194-234)**

def _cargar_callejones(self): \# Lee JSON al iniciar Ronda 2

def _guardar_callejones(self): \# Escribe JSON al terminar Ronda 1

def _celda_es_callejon(self): \# Verifica si una celda es callejón

def _celda_lado_seguido(self): \# Calcula celda vecina hacia el lado seguido

_cargar_callejones(): Al iniciar Ronda 2, lee el archivo callejones_ronda1.json y carga las celdas a evitar.

_guardar_callejones(): Al terminar Ronda 1 (estado FIN), escribe las celdas detectadas como callejones al JSON.

_celda_es_callejon(): Compara una celda contra la lista de callejones conocidos, con margen de ±1 celda (por error de odometría).

_celda_lado_seguido(): Calcula qué celda está al lado que sigue el robot (izquierda). Usa la posición actual (x, y) y la orientación (yaw) para proyectar un paso de 60 cm hacia la izquierda. Esta función es clave: se usa TANTO en Ronda 1 (para registrar la entrada al callejón) COMO en Ronda 2 (para verificar si debe evitarlo). Así ambas rondas apuntan al mismo punto del mapa.

### **Sección 3.13: sector_distance() (líneas 237-254)**

def sector_distance(self, scan, center_deg, on_empty=None, half=SECTOR_HALF):

for d in range(-half, half + 1):

raw_deg = center_deg + FRONT_OFFSET + d

if in_blind(raw_deg): continue

r = scan.ranges[idx]

if rayo_invalido: continue

vals.append(r)

return min(vals) \# distancia mínima del sector

Mide la distancia en un sector angular. Recorre todos los rayos del sector (±12° para lados, ±7° para frente), filtra los inválidos (zona ciega, inf, NaN, fuera de rango), y devuelve el más cercano. Si no hay rayos válidos, devuelve on_empty (0.08 m para lados = "pared muy cerca").

### **Sección 3.14: Callbacks de sensores (líneas 256-289)**

def on_color(self, msg): \# Recibe color de pare_detector.py

def on_scan(self, scan): \# Recibe datos del LiDAR

def on_odom(self, msg): \# Recibe posición del odómetro

on_scan() calcula 5 distancias: frontal, izquierda, derecha, perpendicular a la pared, diagonal a la pared. Las dos últimas son los "2 rayos especiales" para el control PD (mantener paralelo). on_odom() extrae posición (x, y) y orientación (yaw) del cuaternión. on_color() simplemente almacena el último color detectado.

### **Sección 3.15: Control PD — pd_follow() (líneas 291-316)**

def pd_follow(self):

b = self.side_perp \# rayo perpendicular (90°)

a = self.side_diag \# rayo diagonal (50°)

alpha = atan2(a*cos(theta) - b, a*sin(theta)) \# ángulo a pared

dist = b * cos(alpha) \# distancia real

future = dist + LOOKAHEAD * sin(alpha) \# distancia futura

error = future - TARGET_DIST \# error

ang = -(KP * error + KD * deriv) \# corrección PD

Usa 2 rayos (perpendicular a 90° y diagonal a 50°) para calcular: (1) el ángulo del robot respecto a la pared (alpha), (2) la distancia actual perpendicular (dist), (3) la distancia proyectada a 55 cm adelante (future). El error se calcula como la diferencia entre la distancia futura y el objetivo (18 cm). El control PD aplica corrección proporcional (KP) y derivativa (KD) para mantener el robot paralelo y a la distancia correcta.

### **Sección 3.16: Estado FIN (líneas 331-337)**

if self.state == "FIN":

self.stop()

if self.ronda == 1 and not self._callejones_guardados:

self._guardar_callejones()

return

Cuando el robot detecta VERDE (META), entra en estado FIN. Se detiene definitivamente. Si es Ronda 1, guarda los callejones detectados en el archivo JSON antes de parar.

### **Sección 3.17: Estado PARE (líneas 339-349)**

if self.state == "PARE":

if ahora - self.pare_inicio >= PARE_ROJO_SEGUNDOS:

self.state = estado_anterior

self.ignorar_color_hasta = ahora + COOLDOWN

else:

self.stop()

Cuando el robot ve ROJO, se detiene. Espera 3 segundos (PARE_ROJO_SEGUNDOS). Luego vuelve al estado anterior (normalmente FOLLOW). Activa un cooldown de 1.5 segundos para no re-detectar el mismo cartel PARE al retomar el movimiento.

### **Sección 3.18: Reacción al color (líneas 353-371)**

if ahora >= self.ignorar_color_hasta and color_actual != color_anterior:

if nuevo == "VERDE": → state = "FIN"

if nuevo == "ROJO": → state = "PARE"

Detecta cambios de color (por flanco, no por nivel). Solo reacciona cuando el color CAMBIA (de NINGUNO a ROJO, por ejemplo), no cuando se mantiene igual. Esto evita detecciones múltiples del mismo cartel. Verde → FIN (parado definitivo). Rojo → PARE (pausa 3 s). Respeta el cooldown.

### **Sección 3.19: Detección META por LiDAR (líneas 374-379)**

if f > META_OPEN and l > META_OPEN and r > META_OPEN:

self.meta_count += 1

if self.meta_count >= META_FRAMES: → state = "META"

Si TODAS las direcciones (frente, izquierda, derecha) leen más de 1.3 metros durante 15 ciclos seguidos, el robot asume que salió del laberinto a un espacio abierto → META. Es un backup por si la cámara no detecta el verde.

### **Sección 3.20: Estado FOLLOW — Decisión en intersecciones (líneas 388-402)**

near_open = near >= OPEN_THRESH \# ¿hay apertura en mi pared?

front_blocked = f < FRONT_BLOCK \# ¿pared al frente?

if near_open and past_grace: action = "NEAR" \# gira izquierda

elif front_blocked and far_open: action = "FAR" \# gira derecha

elif front_blocked: action = "UTURN" \# media vuelta

Regla de la mano izquierda con 3 prioridades: (1) Si la izquierda se abre (>50 cm) y ya pasó la zona de gracia → gira a la izquierda. (2) Si hay pared al frente (<22 cm) y la derecha se abre → gira a la derecha. (3) Si hay pared al frente y ambos lados cerrados → media vuelta (180°). Si nada aplica → sigue recto con control PD.

### **Sección 3.21: Lógica de memoria — Evitar callejones en Ronda 2 (líneas 405-411)**

if self.ronda == 2 and action == "NEAR":

sig_celda = self._celda_lado_seguido()

if self._celda_es_callejon(sig_celda):

action = None \# NO girar, continuar recto

ANTES de ejecutar un giro hacia la izquierda, calcula qué celda hay al lado y verifica si está en la lista de callejones cargada del JSON. Si es un callejón conocido, cancela el giro (action = None) y el robot sigue recto, evitando entrar al callejón.

### **Sección 3.22: Anti-ruido y ejecución de acciones (líneas 414-452)**

if action == confirm_action: confirm += 1

if confirm >= CONFIRM_TICKS: \# 3 ciclos seguidos

if action == "NEAR":

state = "CREEP" \# avanza 28 cm, luego gira

if ronda == 1: entrada_callejon = _celda_lado_seguido()

elif action == "UTURN":

state = "TURN" \# gira 180°

if ronda == 1 and entrada_callejon != None:

callejones_conocidos.add(entrada_callejon)

Espera 3 ciclos consecutivos (0.15 segundos) con la misma decisión antes de actuar. NEAR: avanza (CREEP) y luego gira. En Ronda 1, registra la celda de entrada. UTURN: gira 180°. En Ronda 1, si había una entrada registrada, confirma que esa celda es un callejón sin salida.

### **Sección 3.23: Estado CREEP (líneas 455-461)**

if self.state == "CREEP":

if self.dist_from(self.creep_ref) < CREEP_DIST:

self.publish(CREEP_SPEED, 0.0) \# avanza recto

else:

self.start_turn(self.next_delta) \# ahora gira

Antes de girar a la izquierda, avanza 28 cm en línea recta (CREEP_DIST). Esto permite que el robot pase la esquina sin que la rueda choque contra la pared. Mide la distancia avanzada con el odómetro. Cuando llega a 28 cm, pasa a TURN.

### **Sección 3.24: Estado TURN (líneas 464-475)**

if self.state == "TURN":

rem = ang_diff(self.turn_target, self.yaw)

if abs(rem) < TURN_TOL: \# llegué al ángulo

state = "FOLLOW"

grace_ref = (x, y) \# activa zona de gracia

else:

speed = clamp(abs(rem) * 1.5, 0.15, TURN_SPEED)

self.publish(0.0, speed * dirección)

Gira hasta alcanzar un ángulo exacto (90° o 180°) medido con el odómetro. La velocidad de giro se reduce suavemente al acercarse al objetivo (frenado suave). Cuando la diferencia angular es menor a 0.05 rad (~3°), para y vuelve a FOLLOW. Activa la zona de gracia (35 cm) para no re-evaluar la misma dirección.

### **Sección 3.25: Estado META (líneas 478-482)**

if self.state == "META":

self.stop()

self.get_logger().info("*** META alcanzada. ***")

Backup de FIN. Si el LiDAR detecta espacio abierto en todas direcciones (salió del laberinto), el robot se detiene.

### **Sección 3.26: Función main() (líneas 485-500)**

def main():

rclpy.init()

node = MazeSolver()

rclpy.spin(node)

Inicia ROS2, crea el nodo MazeSolver, y lo mantiene corriendo. Al hacer Ctrl+C, publica un stop (velocidad 0) antes de cerrar.

# **4. DASHBOARD_NODE.PY — EL PANEL DE MONITOREO**

### **Ubicación y propósito**

**Archivo:** granprix_pkg/granprix_pkg/granprix_pkg/dashboard_node.py

**Líneas:** 1 a 745

**Propósito:** Visualización en tiempo real del estado del robot. NO mueve el robot, solo observa y muestra. Es independiente de los otros nodos.

El dashboard abre una ventana OpenCV que muestra:

• Feed de la cámara con detección de color (HSV + PARE)

• El laberinto real dibujado (rejilla 6×4, paredes MDF) con la posición y trayectoria del robot en vivo

• Tarjetas de métricas: Ronda, Ruta/eficiencia, Señales PARE, Integridad

Se suscribe a los mismos tópicos que maze_solver (/image_raw, /cmd_vel, /odom_raw, /color_detectado, /scan) pero NO publica nada. Es un observador pasivo.

Teclas: 'q'/Esc cierra, 'g' guarda métricas a CSV, 'f' marca un falso PARE.

# **5. SETUP.PY — CONFIGURACIÓN DEL PAQUETE ROS2**

### **Ubicación y propósito**

**Archivo:** granprix_pkg/granprix_pkg/setup.py

**Líneas:** 1 a 27

**Propósito:** Registrar los nodos como ejecutables de ROS2 para que se puedan correr con ros2 run.

entry_points={

'console_scripts': [

'maze_solver = granprix_pkg.maze_solver:main',

'pare_detector = granprix_pkg.pare_detector:main',

'maze_solver_con_memoria = granprix_pkg.maze_solver_con_memoria:main',

],

},

Define 3 ejecutables: maze_solver (original sin memoria), pare_detector (cámara), y maze_solver_con_memoria (con recordatorio de callejones).

# **6. FLUJO COMPLETO DE EJECUCIÓN**

### **Terminal 1: Cámara (driver USB)**

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

Publica frames de la cámara en /image_raw.

### **Terminal 2: Detector de color**

python3 pare_detector.py

Lee /image_raw → detecta ROJO/VERDE → publica en /color_detectado.

### **Terminal 3: Robot navegador**

python3 maze_solver_con_memoria.py

Lee /scan + /odom_raw + /color_detectado → decide → publica /cmd_vel.

### **Terminal 4 (opcional): Dashboard**

python3 dashboard_node.py

Lee todos los tópicos → muestra visualización en ventana OpenCV.

### **Diagrama de comunicación entre nodos:**

usb_cam_node → /image_raw → pare_detector → /color_detectado → maze_solver

LiDAR (YB_Car_Node) → /scan → maze_solver

Encoders (YB_Car_Node) → /odom_raw → maze_solver

maze_solver → /cmd_vel → Motores (YB_Car_Node)

Todos los tópicos → dashboard_node (solo observa)

# **7. SISTEMA DE DOS RONDAS**

### **Ronda 1 — Exploración**

RONDA = 1 (línea 97 de maze_solver_con_memoria.py)

El robot recorre el laberinto con regla de mano izquierda. Cada vez que gira a la izquierda (NEAR), registra la celda de entrada. Si después hace media vuelta (UTURN), confirma que esa celda es un callejón sin salida. Al llegar a META (verde), guarda todas las celdas callejón en callejones_ronda1.json.

### **Ronda 2 — Evitar callejones**

RONDA = 2 (línea 97 de maze_solver_con_memoria.py)

Al iniciar, carga callejones_ronda1.json. Navega igual (regla de mano izquierda), pero ANTES de girar a la izquierda, verifica si la celda destino es un callejón conocido. Si lo es, no ingresa, lo evita. Resultado: ruta más corta porque evita los callejones.

<img src="media/image1.png" style="width:6.5in;height:4.43333in" />
