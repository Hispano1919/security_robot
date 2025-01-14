#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
import cv2
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

import yaml
import random
import rospkg

from APP_config import TOPIC_COMMAND, TOPIC_LOGS, WAYPOINT_PATH, def_waypoints 
from APP_config import STOP_MOVE_CMD, START_MOVE_CMD, NODE_SUCCEED, NODE_FAILURE


class PatrolAreaNode():
    def __init__(self):
        self.active = False
        self.clientAvailable = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd = None
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10) 
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10) 
        self.place = None
        self.log_msg = "[INFO] MOVE STATE: Waiting..."
        
    def start(self, map_name, area_name):
        rospy.init_node('Patrol_Area_Node')
        rospy.spin()
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('security_robot')
        folder_path = self.package_path + "/nav_maps"
        yaml_file = folder_path + "/" + map_name + ".yaml"
        area_file = folder_path + "/" + area_name + ".yaml"
        
        self.execute(yaml_file, area_file, 50)
        
    def stop(self):
        rospy.signal_shutdown("Stopping Patrol Area Node")
        
    def execute(self, ruta_mapa, ruta_yaml, espaciado):
        
        while not rospy.is_shutdown():
            # Solicitar la configuración del área al usuario
            #ruta_mapa = input("Ingrese la ruta del mapa segmentado: ")
            #ruta_yaml = input("Ingrese la ruta del archivo YAML del mapa: ")
            #espaciado = int(input("Ingrese el espaciado entre puntos de patrullaje (en píxeles): "))
        
            """Configurar y patrullar un área específica del mapa."""
            print("Patrullar")
            # Cargar el mapa segmentado y obtener parámetros
            mapa_binario = self.cargar_mapa(ruta_mapa)
            resolucion, origen = self.leer_parametros_mapa(ruta_yaml)

            # Generar puntos de patrullaje
            puntos_patrullaje = self.generar_puntos(mapa_binario, espaciado)

            # Seleccionar 5 puntos aleatorios
            puntos_seleccionados = self.seleccionar_puntos_aleatorios(puntos_patrullaje, cantidad=5)

            # Convertir los puntos seleccionados a coordenadas reales
            puntos_coordenadas = self.convertir_a_coordenadas(puntos_seleccionados, mapa_binario, resolucion, origen)

            # Patrullar los puntos seleccionados
            for x, y in puntos_coordenadas:
                resultado = self.mover_a_goal(x, y)
                if resultado:
                    rospy.loginfo(f"Goal alcanzado en ({x}, {y})")
                else:
                    rospy.loginfo(f"Error alcanzando el goal en ({x}, {y})")

    def cmd_callback(self, msg):
            
        if msg.data == STOP_MOVE_CMD:
            self.active = False
            self.set_current_position_as_goal()
        
    def cargar_mapa(self, ruta_mapa):
        """Cargar el mapa segmentado en formato PGM y binarizarlo."""
        print("Cargar Mapa")
        mapa = cv2.imread(ruta_mapa, cv2.IMREAD_GRAYSCALE)
        if mapa is None:
            raise FileNotFoundError(f"No se pudo cargar el mapa en {ruta_mapa}")
        _, mapa_binario = cv2.threshold(mapa, 200, 255, cv2.THRESH_BINARY)
        return mapa_binario

    def leer_parametros_mapa(self, ruta_yaml):
        """Leer resolución y origen del mapa desde el archivo YAML."""
        print("Leer yaml")
        with open(ruta_yaml, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        resolucion = yaml_data['resolution']
        origen = yaml_data['origin']  # Origen en coordenadas del mundo real (x, y, theta)
        return resolucion, (origen[0], origen[1])

    def generar_puntos(self, mapa_binario, espaciado, distancia_minima=5):
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

    def seleccionar_puntos_aleatorios(self, puntos, cantidad=5):
        """Seleccionar una cantidad específica de puntos aleatorios de la lista."""
        print("Puntos aleatorios")
        if len(puntos) > cantidad:
            return random.sample(puntos, cantidad)  # Selecciona 'cantidad' puntos aleatorios
        else:
            return puntos  # Si hay menos puntos de los necesarios, retornar todos

    def convertir_a_coordenadas(self, puntos, mapa_binario, resolucion, origen):
        """Convertir puntos de píxeles en el mapa a coordenadas reales en el marco 'map'."""
        print("Coordenadas")
        puntos_coordenadas = []
        for px, py in puntos:
            x = px * resolucion + origen[0]
            y = (mapa_binario.shape[0] - py) * resolucion + origen[1]
            puntos_coordenadas.append((x, y))
        return puntos_coordenadas

    def mover_a_goal(self, x, y):
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

    def set_current_position_as_goal(self):
        """Establece la posición actual del robot como un nuevo objetivo."""
        rospy.loginfo("Obteniendo la posición actual del robot...")

        # Asegúrate de que la pose actual esté disponible
        if self.current_pose is None:
            rospy.logwarn("No se ha recibido la posición actual del robot.")
            return

        # Crear un nuevo objetivo basado en la posición actual
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # El marco debe ser consistente con tu sistema
        goal.target_pose.header.stamp = rospy.Time.now()

        # Usar la posición y orientación actual del robot
        goal.target_pose.pose.position.x = self.current_pose.position.x
        goal.target_pose.pose.position.y = self.current_pose.position.y
        goal.target_pose.pose.position.z = self.current_pose.position.z
        goal.target_pose.pose.orientation.x = self.current_pose.orientation.x
        goal.target_pose.pose.orientation.y = self.current_pose.orientation.y
        goal.target_pose.pose.orientation.z = self.current_pose.orientation.z
        goal.target_pose.pose.orientation.w = self.current_pose.orientation.w

        # Enviar el nuevo objetivo al cliente de navegación
        self.client.send_goal(goal)
        rospy.loginfo("Nuevo objetivo enviado: mantener la posición actual.")

        # Opcional: Esperar a que el cliente confirme que el objetivo ha sido alcanzado
        self.client.wait_for_result()
        rospy.loginfo("Robot detenido en la posición actual.")
        self.goal_cancel = True