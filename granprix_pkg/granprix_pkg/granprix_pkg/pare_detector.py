#!/usr/bin/env python3
"""
pare_detector.py - Nodo independiente de vision (rojo/verde)
---------------------------------------------------------------
NO mueve el carrito, NO sabe nada del laberinto ni del laser.
Su unico trabajo es:
  1. Suscribirse a la imagen de la camara (/image_raw)
  2. Detectar si hay ROJO o VERDE en la imagen
  3. Publicar el resultado como texto en el topico /color_detectado

maze_solver.py se suscribe a ese topico para saber que hacer,
pero este archivo no sabe nada de maze_solver.py -> estan
desacoplados, cada uno puede correr o modificarse sin tocar al otro.

Como correrlo (en su propia terminal, con la camara ya activa):
    python3 pare_detector.py
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# --------------------------------------------------------------------
# CAMBIA ESTO si tu topico de camara se llama distinto
# --------------------------------------------------------------------
TOPIC_IMAGEN     = "/image_raw"
TOPIC_COLOR      = "/color_detectado"   # topico de salida que escucha maze_solver.py
MIN_AREA_COLOR   = 800

# Solo nos interesan ROJO y VERDE
COLOR_RANGES = {
    "ROJO": [
        (np.array([0, 120, 70]),   np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    "VERDE": [
        (np.array([36, 80, 60]), np.array([85, 255, 255])),
    ],
}


def detectar_color(frame_hsv):
    """Devuelve 'ROJO', 'VERDE' o 'NINGUNO'."""
    mejor_color = "NINGUNO"
    mejor_area = 0
    for nombre, rangos in COLOR_RANGES.items():
        mascara_total = None
        for bajo, alto in rangos:
            m = cv2.inRange(frame_hsv, bajo, alto)
            mascara_total = m if mascara_total is None else cv2.bitwise_or(mascara_total, m)
        mascara_total = cv2.erode(mascara_total, None, iterations=2)
        mascara_total = cv2.dilate(mascara_total, None, iterations=2)
        area = cv2.countNonZero(mascara_total)
        if area > mejor_area:
            mejor_area = area
            mejor_color = nombre
    if mejor_area >= MIN_AREA_COLOR:
        return mejor_color
    return "NINGUNO"


class ParedetectorNode(Node):
    def __init__(self):
        super().__init__('pare_detector')

        self.bridge = CvBridge()
        self.color_anterior = None  # solo para no llenar la consola de logs repetidos

        self.suscripcion = self.create_subscription(
            Image, TOPIC_IMAGEN, self.callback_imagen, 10)

        self.publicador = self.create_publisher(String, TOPIC_COLOR, 10)

        self.get_logger().info(
            f"pare_detector iniciado. Escuchando {TOPIC_IMAGEN}, publicando en {TOPIC_COLOR}")

    def callback_imagen(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            return

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color = detectar_color(frame_hsv)

        # Publicamos SIEMPRE (cada frame), maze_solver.py decide que hacer con eso
        salida = String()
        salida.data = color
        self.publicador.publish(salida)

        # Solo para consola: log cuando el color cambia
        if color != self.color_anterior:
            self.get_logger().info(f"Color detectado: {color}")
            self.color_anterior = color

        # Ventana opcional para ver que esta viendo la camara
        # (si el VNC no tiene entorno grafico y da error, comenta estas 2 lineas)
        cv2.imshow("pare_detector - camara", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    nodo = ParedetectorNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
