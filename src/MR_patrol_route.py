#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String

import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

import rospkg

import csv

from APP_config import TOPIC_COMMAND, TOPIC_LOGS, WAYPOINT_PATH, def_waypoints 
from APP_config import STOP_MOVE_CMD, STOP_MOVE_NODE, START_MOVE_CMD, NODE_SUCCEED, NODE_FAILURE, PACK_NAME, MAP_NAME

class PatrolRouteNode():
    def __init__(self):
        self.is_active = True
        self.clientAvailable = True
        self.stop_node = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd = None
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10) 
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10) 
        self.place = None
        
        self.start()
        
    def start(self):
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(PACK_NAME)
        folder_path = package_path + "/output_files"
        csv_file = folder_path + "/" + MAP_NAME + ".csv"
        
        self.execute(csv_file)
        
    def stop(self):
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.ACTIVE:
            self.set_current_position_as_goal()
            while not self.goal_cancel:
                rospy.sleep(0.1)
                
        self.log_pub.publish("[INFO] PATROL NODE: Stopped patrol route node")
        self.cmd_sub.unregister()  
        self.is_active = False
        
    def cmd_callback(self, msg):
            
        if msg.data == STOP_MOVE_CMD:
            self.is_active = False
            self.stop()
            
    def execute(self, csv_path):
        # Leer los centroides desde el archivo CSV
        centroides = self.leer_centroides(csv_path)

        # Mover el robot a cada uno de los centroides
        self.mover_a_centroides(centroides)
    
        self.stop()
        self.stop_node = True
        
    def mover_a_goal(self, x,y):            
        # Si el cliente move_base esta activo
        if self.clientAvailable:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0  # Orientación neutra (sin rotación)

            self.client.send_goal(goal)
            self.client.wait_for_result()
        
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return 'succeeded'
            else:        
                return 'aborted'
        else:
            return 'aborted' 
        
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
            self.log_pub.publish("[INFO] PATROL NODE: Patroling route...")
            result = self.mover_a_goal(x, y)
            
            if result:
                self.log_pub.publish("[INFO] PATROL NODE: Centroid reached")
                rospy.loginfo(f"Llegamos al paso {idx+1}")
            else:
                self.log_pub.publish("[INFO] PATROL NODE: Centroid not reached")
                rospy.logwarn(f"Falló al llegar al paso {idx+1}")
            
            if self.is_active == False:
                break
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

def main():
    rospy.init_node('PatrolRouteNode')
    
    moveNode = PatrolRouteNode()

    if moveNode.stop_node:
        moveNode.log_pub.publish("[INFO] PATROL NODE: Node closed")
        moveNode.cmd_pub.publish(STOP_MOVE_NODE)
        rospy.signal_shutdown("MoveNode_Stop")
        
    rospy.spin()
if __name__ == '__main__':
    main()
