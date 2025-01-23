#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
import rospkg

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import re

import argparse

from APP_config import TOPIC_COMMAND, TOPIC_LOGS, WAYPOINT_PATH, MAP_NAME, def_waypoints 
from APP_config import STOP_MOVE_CMD, START_MOVE_CMD, STOP_MOVE_NODE, PACK_NAME

class QRMoveNode():
    def __init__(self):
        
        self.is_active = True
        self.stop_node = False
        self.goal_cancel = False
        self.clientAvailable = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd = None
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        self.cmd_sub = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        
        # Usar argparse para recoger el argumento --place
        parser = argparse.ArgumentParser(description="Mover al robot a un lugar específico.")
        parser.add_argument('--place', type=str, required=True, help="El nombre del lugar al que moverse")
        args = parser.parse_args()

        # Asignar el valor del argumento a self.place
        self.waypoint_name = args.place
        
        print(self.waypoint_name)
        self.log_msg = "[INFO] QRMOVE NODE: Waiting..."
        
        self.execute()

        # Espera un máximo de 10 segundos para que el servidor esté disponible
        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            self.clientAvailable = False
            rospy.logerr("'move_base' server not available")

    def execute(self):
        self.log_pub.publish("[INFO] QRMOVE NODE: Executing node")
        
        self.start()
        
        self.move_to_qrWaypoint()
                    
        self.stop()
            
        self.stop_node = True
        
    def start(self):
        
        
        self.timer = rospy.Timer(rospy.Duration(5), self.publish_message)
        
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.log_pub.publish("[INFO] QRMOVE NODE: Started QR Waypoint node")
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(PACK_NAME)
        qrfolder_path = package_path + "/output_files"
        self.log_file_path = qrfolder_path + "/" + MAP_NAME + ".qrlog"
        self.update_waypoints_from_file(self.log_file_path, def_waypoints)

        self.waypoint = None
        self.waypoint = next((w for w in def_waypoints if w[0] == self.waypoint_name), None)
        
        # Si no existe, se cancela la orden de movimiento
        if self.waypoint == None:
            self.log_pub.publish(f"[INFO] QRMOVE NODE: {self.waypoint_name} not found")
            self.stop()
        
    def stop(self):
        
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.ACTIVE:
            self.set_current_position_as_goal()
            while not self.goal_cancel:
                rospy.sleep(0.1)
                
        self.log_pub.publish("[INFO] QRMOVE NODE: Stopped QR Waypoint node")
        self.cmd_sub.unregister()  
        self.timer.shutdown()
        self.is_active = False
  
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
                        self.log_pub.publish(f"[INFO] QRMOVE NODE: Updated waypoint list from {filename}")
                        break

                # Si no se encuentra, añadir uno nuevo
                if not updated:
                    waypoints.append([name, position, orientation])

    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)

    def cmd_callback(self, msg):
            
        if msg.data == STOP_MOVE_CMD:
            self.is_active = False
            self.stop()
            
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
        
    def move_to_qrWaypoint(self):
        self.log_msg = None
        
        self.log_msg = f"[INFO] QRMOVE NODE: Moving to {self.waypoint_name}..."
        output = self.move_to_goal(self.waypoint)
        
        if output == "succeeded":
            self.log_pub.publish(f"[INFO] QRMOVE NODE: {self.waypoint_name} reached")
            self.stop()
        elif output == "aborted":
            self.log_pub.publish(f"[INFO] QRMOVE NODE: {self.waypoint_name} not found")
            self.stop()
           
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

def main():
    rospy.init_node('QRWaypointMoveNode')
    
    moveNode = QRMoveNode()

    if moveNode.stop_node:
        moveNode.log_pub.publish("[INFO] QRMOVE NODE: Node closed")
        moveNode.cmd_pub.publish(STOP_MOVE_NODE)
        rospy.signal_shutdown("MoveNode_Stop")
        
    rospy.spin()
if __name__ == '__main__':
    main()

