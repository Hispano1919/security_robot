
import cv2
import numpy as np
import rospy
import actionlib
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

def cargar_mapa(ruta_mapa):
    """Cargar el mapa segmentado en formato PGM y binarizarlo."""
    mapa = cv2.imread(ruta_mapa, cv2.IMREAD_GRAYSCALE)
    if mapa is None:
        raise FileNotFoundError(f"No se pudo cargar el mapa en {ruta_mapa}")
    _, mapa_binario = cv2.threshold(mapa, 200, 255, cv2.THRESH_BINARY)
    return mapa_binario

def leer_parametros_mapa(ruta_yaml):
    """Leer resolución y origen del mapa desde el archivo YAML."""
    with open(ruta_yaml, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    resolucion = yaml_data['resolution']
    origen = yaml_data['origin']  # Origen en coordenadas del mundo real (x, y, theta)
    return resolucion, (origen[0], origen[1])

def convertir_a_coordenadas(px, py, mapa_binario, resolucion, origen):
    """
    Convertir las coordenadas de un píxel en el mapa a coordenadas reales en el marco 'map'.
    """
    x = px * resolucion + origen[0]
    y = origen[1] + (mapa_binario.shape[0] - py) * resolucion
    return x, y

def calcular_orientacion(x1, y1, x2, y2):
    """
    Calcular la orientación (ángulo theta) entre dos puntos (x1, y1) y (x2, y2).
    """
    delta_x = x2 - x1
    delta_y = y2 - y1
    return math.atan2(delta_y, delta_x)

def mover_a_goal(x, y, theta):
    """Enviar un goal al robot para moverse a una ubicación específica con una orientación."""
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(theta / 2)
    goal.target_pose.pose.orientation.w = math.cos(theta / 2)

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def seleccionar_pixel_y_mover(ruta_mapa, ruta_yaml):
    """Permitir al usuario seleccionar un píxel en el mapa y mover el robot a esa ubicación con orientación."""
    # Cargar el mapa y parámetros
    mapa = cv2.imread(ruta_mapa)
    if mapa is None:
        raise FileNotFoundError(f"No se pudo cargar el mapa en {ruta_mapa}")
    mapa_binario = cv2.cvtColor(mapa, cv2.COLOR_BGR2GRAY)
    resolucion, origen = leer_parametros_mapa(ruta_yaml)

    coordenada_inicio = None  # Guardamos la primera selección (ubicación)
    mapa_original = mapa.copy()  # Copia del mapa original para restablecerlo

    # Guardar la imagen binarizada como archivo .png para revisión
    cv2.imwrite("imagen_binaria_casa_3.png", mapa_binario)
    print("Imagen binaria guardada como imagen_binaria_casa_3.png")

    def click_event(event, x, y, flags, param):
        """Evento para manejar clics en la imagen."""
        nonlocal coordenada_inicio, mapa
        if event == cv2.EVENT_LBUTTONDOWN:
            # Validar si el píxel seleccionado es blanco (valor 255)
            if mapa_binario[y, x] >= 240:
                if coordenada_inicio is None:
                    # Primer clic: seleccionar ubicación
                    coordenada_inicio = (x, y)
                    coord_x, coord_y = convertir_a_coordenadas(x, y, mapa_binario, resolucion, origen)
                    print(f"Ubicación seleccionada (en metros): x = {coord_x}, y = {coord_y}")
                    cv2.circle(mapa, (x, y), 5, (0, 255, 0), -1)  # Marcar el primer punto
                else:
                    # Segundo clic: seleccionar orientación
                    coord_x_inicio, coord_y_inicio = convertir_a_coordenadas(coordenada_inicio[0], coordenada_inicio[1], mapa_binario, resolucion, origen)
                    coord_x_final, coord_y_final = convertir_a_coordenadas(x, y, mapa_binario, resolucion, origen)

                    # Calcular la orientación en radianes
                    theta = calcular_orientacion(coord_x_inicio, coord_y_inicio, coord_x_final, coord_y_final)
                    print(f"Orientación seleccionada (en radianes): {theta}")

                    # Mover el robot al goal con orientación
                    resultado = mover_a_goal(coord_x_inicio, coord_y_inicio, theta)
                    if resultado:
                        rospy.loginfo(f"Goal alcanzado en ({coord_x_inicio}, {coord_y_inicio}) con orientación {theta}")
                        
                        # Eliminar el punto verde al alcanzar el objetivo
                        mapa = mapa_original.copy()  # Restaurar el mapa original sin el punto verde
                        cv2.imshow('Mapa', mapa)
                    else:
                        rospy.loginfo(f"Error alcanzando el goal en ({coord_x_inicio}, {coord_y_inicio})")

                    # Restablecer la primera selección
                    coordenada_inicio = None
            else:
                print("¡Solo se pueden seleccionar píxeles blancos!")

            # Mostrar el mapa actualizado
            cv2.imshow('Mapa', mapa)

    # Mostrar el mapa y esperar la selección del usuario
    cv2.imshow('Mapa', mapa)
    cv2.setMouseCallback('Mapa', click_event)

    print("Haga clic en un punto blanco del mapa para seleccionar la ubicación y otro para la orientación. Presione 'q' para salir.")
    while True:
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Inicializar el nodo de ROS
    rospy.init_node('seleccionar_pixel_y_mover')

    # Solicitar rutas del mapa y YAML al usuario
    ruta_mapa = input("Ingrese la ruta del mapa segmentado: ")
    ruta_yaml = input("Ingrese la ruta del archivo YAML del mapa: ")

    # Ejecutar la selección del píxel y mover al robot
    seleccionar_pixel_y_mover(ruta_mapa, ruta_yaml)


