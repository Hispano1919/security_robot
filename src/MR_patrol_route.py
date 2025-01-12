import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String

import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

import rospkg

import csv

from APP_main import TOPIC_COMMAND, TOPIC_LOGS, WAYPOINT_PATH, def_waypoints 
from APP_main import STOP_MOVE_CMD, START_MOVE_CMD, NODE_SUCCEED, NODE_FAILURE

class PatrolRouteNode():
    def __init__(self):
        self.active = False
        self.clientAvailable = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd = None
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10) 
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10) 
        self.place = None
        self.log_msg = "[INFO] MOVE STATE: Waiting..."
        
    def start(self, map_name):
        rospy.init_node('Patrol_Route_Node')
        rospy.spin()
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('security_robot')
        folder_path = package_path + "/output_files"
        csv_file = folder_path + "/" + map_name + ".csv"
        
        self.execute(csv_file)
        
    def stop(self):
        rospy.signal_shutdown("Stopping Patrol Area Node")
        
    def cmd_callback(self, msg):
            
        if msg.data == STOP_MOVE_CMD:
            self.active = False
            self.set_current_position_as_goal()
            
    def execute(self, csv_path):
        # Leer los centroides desde el archivo CSV
        centroides = self.leer_centroides(csv_path)

        # Mover el robot a cada uno de los centroides
        self.mover_a_centroides(centroides)
    
    def mover_a_goal(self, x, y):
        """Enviar un goal al robot para moverse a una ubicación específica."""
        # Crear un cliente de acción para el servidor 'move_base'
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # Crear un objetivo (goal) para el robot
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Orientación neutra (sin rotación)

        # Enviar el goal al servidor 'move_base'
        client.send_goal(goal)
        client.wait_for_result()

        # Retornar el resultado del movimiento
        return client.get_result()

    def leer_centroides(self, csv_path):
        """Leer los centroides desde un archivo CSV."""
        centroides = []
        with open(csv_path, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Saltar la cabecera
            for row in reader:
                x = float(row[0])  # Coordenada X en metros
                y = float(row[1])  # Coordenada Y en metros
                centroides.append((x, y))
        return centroides

    def mover_a_centroides(self, centroides):
        """Mover el robot a cada uno de los centroides."""
        for idx, (x, y) in enumerate(centroides):
            rospy.loginfo(f"Moviendo al paso {idx+1} a las coordenadas: ({x}, {y})")
            result = self.mover_a_goal(x, y)
            
            if result:
                rospy.loginfo(f"Llegamos al paso {idx+1}")
            else:
                rospy.logwarn(f"Falló al llegar al paso {idx+1}")
            
            # Esperar un poco antes de moverse al siguiente objetivo
            rospy.sleep(2)  # Ajusta el tiempo de espera según sea necesario

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
