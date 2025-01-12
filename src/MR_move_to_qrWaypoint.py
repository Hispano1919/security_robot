import rospy
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import Point
from std_msgs.msg import String
import cv2
import numpy as np
import mediapipe as mp

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import re

from APP_main import TOPIC_COMMAND, TOPIC_LOGS, WAYPOINT_PATH, def_waypoints 
from APP_main import STOP_MOVE_CMD, START_MOVE_CMD, NODE_SUCCEED, NODE_FAILURE

class QRMoveNode():
    def __init__(self):
        
        self.active = False
        self.clientAvailable = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd = None
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10) 
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10) 
        self.place = None
        self.log_msg = "[INFO] MOVE STATE: Waiting..."

        # Espera un máximo de 10 segundos para que el servidor esté disponible
        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            self.clientAvailable = False
            rospy.logerr("'move_base' server not available")

    def update_waypoints_from_file(self, filename, waypoints):
        """
        Funcion para leer el fichero de waypoints y actualizar la lista
        """
        with open(filename, 'r') as file:
            lines = file.readlines()

        # Expresión regular para parsear las líneas
        pattern = re.compile(r"^(.*?), \((.*?), (.*?), (.*?)\), \((.*?), (.*?), (.*?), (.*?)\)$")

        for line in lines:
            match = pattern.match(line.strip())
            if match:
                name = match.group(1).strip()
                position = tuple(map(float, match.group(2, 3, 4)))
                orientation = tuple(map(float, match.group(5, 6, 7, 8)))

                # Verificar si el nombre ya está en la lista
                updated = False
                for waypoint in waypoints:
                    if waypoint[0] == name:
                        waypoint[1] = position
                        waypoint[2] = orientation
                        updated = True
                        self.log_pub.publish(f"[INFO] MOVE STATE: Updated waypoint list from {filename}")
                        break

                # Si no se encuentra, añadir uno nuevo
                if not updated:
                    waypoints.append([name, position, orientation])

    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)

    def cmd_callback(self, msg):

        if ":" in msg.data: 
            cmd, self.place = msg.data.split(":")
        else:
            cmd = msg.data
            
        if cmd == STOP_MOVE_CMD:
            self.active = False
            self.set_current_position_as_goal()
            
        elif START_MOVE_CMD in cmd:
            self.active == True
            self.move_to_qrWaypoint()

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
        self.goal_cancel = False
        
    def move_to_qrWaypoint(self):
        
        self.log_msg = None
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        self.timer = rospy.Timer(rospy.Duration(10), self.publish_message)

        # Obtiene el waypoint deseado
        self.update_waypoints_from_file(WAYPOINT_PATH, def_waypoints)
        waypoint_name = self.place
        print(def_waypoints)
        waypoint = None
        waypoint = next((w for w in def_waypoints if w[0] == waypoint_name), None)
        
        # Si no existe, se cancela la orden de movimiento
        if waypoint == None:
            self.log_pub.publish(f"[INFO] MOVE STATE: {waypoint_name} not found")
            self.timer.shutdown()
            return 'aborted'
        
        self.log_msg = f"[INFO] MOVE STATE: Moving to {waypoint_name}..."
        output = self.move_to_goal(waypoint)
        
        if output == "succeeded":
            self.log_pub.publish(f"[INFO] MOVE STATE: {waypoint_name} reached")
            self.cmd_pub.publish(NODE_SUCCEED)
        elif output == "aborted":
            self.log_pub.publish(f"[INFO] MOVE STATE: {waypoint_name} not found")
            self.cmd_pub.publish(NODE_FAILURE)
           
    def move_to_goal(self, pose):            
        # Si el cliente move_base esta activo
        if self.clientAvailable:
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.pose.position.x = pose[1][0]
            goal_pose.target_pose.pose.position.y = pose[1][1]
            goal_pose.target_pose.pose.position.z = 0.0
            goal_pose.target_pose.pose.orientation.x = pose[2][0]
            goal_pose.target_pose.pose.orientation.y = pose[2][1]
            goal_pose.target_pose.pose.orientation.z = pose[2][2]
            goal_pose.target_pose.pose.orientation.w = pose[2][3]

            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
        
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                self.timer.shutdown()
                return 'succeeded'
            else:        
                self.timer.shutdown()
                return 'aborted'
        else:
            self.timer.shutdown()
            return 'aborted'      



