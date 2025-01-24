#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import numpy as np
import rospkg

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import re

import argparse

from APP_config import TOPIC_COMMAND, TOPIC_LOGS, TOPIC_AMCLPOS 
from APP_config import STOP_MOVE_CMD, STOP_MOVE_NODE

class MoveToPointNode():
    def __init__(self):
        self.is_active = True
        self.stop_node = False
        self.goal_cancel = False
        self.clientAvailable = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        self.cmd_sub = rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        self.pose_sub = rospy.Subscriber(TOPIC_AMCLPOS, PoseWithCovarianceStamped, self.update_pose)
        
        self.current_pose = None
        # Usar argparse para recoger los argumentos x, y, orientacion
        parser = argparse.ArgumentParser(description="Mover al robot a una coordenada específica.")
        parser.add_argument('--x', type=float, required=True, help="Coordenada X del destino")
        parser.add_argument('--y', type=float, required=True, help="Coordenada Y del destino")
        parser.add_argument('--w', type=float, required=True, help="Orientación del destino (en radianes)")
        args = parser.parse_args()

        self.target_x = float(args.x)
        self.target_y = float(args.y)
        self.target_orientation = float(args.w)
        
        self.log_msg = "[INFO] MOVE NODE: Waiting..."
        self.execute()

    def execute(self):
        self.log_pub.publish("[INFO] MOVE NODE: Executing node")
        self.move_to_coordinates()
        self.stop()
        self.stop_node = True

    def stop(self):
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.ACTIVE:
            self.set_current_position_as_goal()
            while not self.goal_cancel:
                rospy.sleep(0.1)
                
        self.log_pub.publish("[INFO] MOVE NODE: Stopped patrol route node")
        self.cmd_sub.unregister()  
        self.is_active = False
        
    def update_pose(self, msg):
        # Actualizar la posición y orientación del robot
        self.current_pose = msg.pose.pose
        
    def publish_message(self, event):
        self.log_pub.publish(self.log_msg)

    def cmd_callback(self, msg):
        if msg.data == STOP_MOVE_CMD:
            self.is_active = False
            self.stop()

    def move_to_coordinates(self):
        if self.clientAvailable:
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.header.stamp = rospy.Time.now()

            goal_pose.target_pose.pose.position.x = self.target_x
            goal_pose.target_pose.pose.position.y = self.target_y
            goal_pose.target_pose.pose.position.z = 0.0
            goal_pose.target_pose.pose.orientation.x = 0.0
            goal_pose.target_pose.pose.orientation.y = 0.0
            goal_pose.target_pose.pose.orientation.z = 0.0
            goal_pose.target_pose.pose.orientation.w = 1.0 

            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
        
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
    rospy.init_node('MoveToCoordinateNode')
    moveNode = MoveToPointNode()

    if moveNode.stop_node:
        moveNode.log_pub.publish("[INFO] MOVE NODE: Node closed")
        moveNode.cmd_pub.publish(STOP_MOVE_NODE)
        rospy.signal_shutdown("MoveNode_Stop")
    
    rospy.spin()

if __name__ == '__main__':
    main()

