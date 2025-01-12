

import cv2
import numpy as np
import rospy
import actionlib
import yaml
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def cargar_mapa(ruta_mapa):
    """Cargar el mapa segmentado en formato PGM y binarizarlo."""
    print("Cargar Mapa")
    mapa = cv2.imread(ruta_mapa, cv2.IMREAD_GRAYSCALE)
    if mapa is None:
        raise FileNotFoundError(f"No se pudo cargar el mapa en {ruta_mapa}")
    _, mapa_binario = cv2.threshold(mapa, 200, 255, cv2.THRESH_BINARY)
    return mapa_binario

def leer_parametros_mapa(ruta_yaml):
    """Leer resolución y origen del mapa desde el archivo YAML."""
    print("Leer yaml")
    with open(ruta_yaml, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    resolucion = yaml_data['resolution']
    origen = yaml_data['origin']  # Origen en coordenadas del mundo real (x, y, theta)
    return resolucion, (origen[0], origen[1])

def generar_puntos(mapa_binario, espaciado, distancia_minima=5):
    """Generar puntos de patrullaje distribuidos uniformemente en el área navegable, evitando zonas cercanas a píxeles negros."""
    print("Generando puntos de patrullaje")
    altura, ancho = mapa_binario.shape
    puntos = []
    
    # Crear una máscara para los píxeles inaccesibles (margen de 5 píxeles alrededor de píxeles negros)
    mapa_inaccesible = np.zeros_like(mapa_binario)
    for y in range(1, altura-1):
        for x in range(1, ancho-1):
            if mapa_binario[y, x] == 0:  # Si es un píxel negro (inaccesible)
                mapa_inaccesible[max(0, y-distancia_minima):min(altura, y+distancia_minima),
                                 max(0, x-distancia_minima):min(ancho, x+distancia_minima)] = 255
    
    # Generar puntos en áreas accesibles
    for y in range(0, altura, espaciado):
        for x in range(0, ancho, espaciado):
            if mapa_binario[y, x] == 255 and mapa_inaccesible[y, x] == 0:  # Si es navegable y no está cerca de un píxel negro
                puntos.append((x, y))
    
    return puntos

def seleccionar_puntos_aleatorios(puntos, cantidad=5):
    """Seleccionar una cantidad específica de puntos aleatorios de la lista."""
    print("Puntos aleatorios")
    if len(puntos) > cantidad:
        return random.sample(puntos, cantidad)  # Selecciona 'cantidad' puntos aleatorios
    else:
        return puntos  # Si hay menos puntos de los necesarios, retornar todos

def convertir_a_coordenadas(puntos, mapa_binario, resolucion, origen):
    """Convertir puntos de píxeles en el mapa a coordenadas reales en el marco 'map'."""
    print("Coordenadas")
    puntos_coordenadas = []
    for px, py in puntos:
        x = px * resolucion + origen[0]
        y = (mapa_binario.shape[0] - py) * resolucion + origen[1]
        puntos_coordenadas.append((x, y))
    return puntos_coordenadas

def mover_a_goal(x, y):
    """Enviar un goal al robot para moverse a una ubicación específica."""
    print("Movimiento")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def patrullar_area(ruta_mapa, ruta_yaml, espaciado):
    """Configurar y patrullar un área específica del mapa."""
    print("Patrullar")
    # Cargar el mapa segmentado y obtener parámetros
    mapa_binario = cargar_mapa(ruta_mapa)
    resolucion, origen = leer_parametros_mapa(ruta_yaml)

    # Generar puntos de patrullaje
    puntos_patrullaje = generar_puntos(mapa_binario, espaciado)

    # Seleccionar 5 puntos aleatorios
    puntos_seleccionados = seleccionar_puntos_aleatorios(puntos_patrullaje, cantidad=5)

    # Convertir los puntos seleccionados a coordenadas reales
    puntos_coordenadas = convertir_a_coordenadas(puntos_seleccionados, mapa_binario, resolucion, origen)

    # Patrullar los puntos seleccionados
    for x, y in puntos_coordenadas:
        resultado = mover_a_goal(x, y)
        if resultado:
            rospy.loginfo(f"Goal alcanzado en ({x}, {y})")
        else:
            rospy.loginfo(f"Error alcanzando el goal en ({x}, {y})")

if __name__ == '__main__':
    # Inicializar el nodo de ROS
    rospy.init_node('patrullaje_automatico')

    while not rospy.is_shutdown():
        # Solicitar la configuración del área al usuario
        ruta_mapa = input("Ingrese la ruta del mapa segmentado: ")
        ruta_yaml = input("Ingrese la ruta del archivo YAML del mapa: ")
        espaciado = int(input("Ingrese el espaciado entre puntos de patrullaje (en píxeles): "))

        # Patrullar el área especificada
        patrullar_area(ruta_mapa, ruta_yaml, espaciado)

