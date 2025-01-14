#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import argparse

from APP_config import TOPIC_COMMAND, TOPIC_LOGS, TOPIC_VEL, TOPIC_PERSONPOSE, MOVE_CLOSER_CMD, MOVE_AWAY_CMD, RESET_DIST_CMD, IDENTIFY_CMD 
from APP_config import START_DETECTION_CMD, STOP_DETECTION_CMD, START_FOLLOW_CMD, STOP_FOLLOW_CMD, NODE_SUCCEED, NODE_FAILURE, STOP_FOLLOW_NODE
from APP_config import MAX_VSPEED, MAX_WSPEED, ANGULAR_GAIN, LINEAR_GAIN, TARGET_DISTANCE, DISTANCE_ERROR, CENTER_TOLERANCE_X

class FollowPersonNode():
    def __init__(self):

        self.is_active = True
        self.stop_node = False
        # Publicador para el tópico de velocidad
        self.cmd_vel_pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.cmd_vision_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        self.cmd_sub = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        
        self.log_msg = "[INFO] FOLLOW NODE: Waiting..."
        # VARIABLES
        self.last_error = 1
        self.followPerson = True
        self.cmd = None
        self.dynamicDist = 0
        self.last_twist = Twist()
        self.last_twist.angular.z = 0
        self.last_twist.linear.x = 0
        self.person_pose = False
        
        parser = argparse.ArgumentParser(description="Script for robot control")
        parser.add_argument("--identify", type=str, help="Specify the map name")
        args = parser.parse_args()

        self.id_flag = args.identify
        print(self.id_flag)
        self.execute()
        
    def execute(self):
        rate = rospy.Rate(0.1)
        self.log_pub.publish("[INFO] FOLLOW NODE: Executing node")
        self.start()
        
        while not rospy.is_shutdown() and self.is_active:
            if not self.person_pose:
                self.cmd_vision_pub.publish(START_DETECTION_CMD)
                
            rate.sleep()
        
        self.stop_node = True
        
    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)

    def start(self):

        self.timer = rospy.Timer(rospy.Duration(5), self.publish_message)
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.person_sub = rospy.Subscriber(TOPIC_PERSONPOSE, Point, self.person_pose_callback)
        self.cmd_vision_pub.publish(START_DETECTION_CMD)
        self.log_pub.publish("[INFO] FOLLOW NODE: Node started")
            

    def stop(self):
        self.log_pub.publish("[INFO] FOLLOW NODE: Node stopped")
        self.person_sub.unregister()
        self.cmd_sub.unregister()

        self.dynamicDist = 0
        self.last_error = 0  
        
        self.cmd_vision_pub.publish(STOP_DETECTION_CMD)
        self.timer.shutdown()
        
        self.is_active = False
    
    def cmd_callback(self, msg):
   
        if msg.data == MOVE_CLOSER_CMD:
            self.dynamicDist = self.dynamicDist - 0.5
        elif msg.data == MOVE_AWAY_CMD:
            self.dynamicDist = self.dynamicDist + 0.5
        elif msg.data == RESET_DIST_CMD:
            self.dynamicDist = 0
        elif msg.data == START_FOLLOW_CMD:
            self.start()
        elif msg.data == STOP_FOLLOW_CMD:
            self.stop()
            
        self.cmd = msg.data

    def person_pose_callback(self, msg):
        """Callback para recibir la posición del pixel desde /person_pose."""
        if self.cmd == STOP_FOLLOW_CMD:
            self.person_sub.unregister()  
        
        self.person_pose = True
        error_x = msg.x
        pixel_depth = msg.y
        
        if error_x == -1 and pixel_depth == -1:   # Person detection has not detected a person
            twist = Twist()
            twist.linear.x = 0

            if self.last_error > 0:
                twist.angular.z = -MAX_WSPEED 
            else:
                twist.angular.z = MAX_WSPEED 
            
            self.log_msg = "[INFO] FOLLOW NODE: Searching for person..."
            self.cmd_vel_pub.publish(twist)

        # SI HAY UNA PERSONA EN LA IMAGEN
        else:
            # Obtener la profundidad en el pixel correspondiente
            self.log_msg = "[INFO] FOLLOW NODE: Person found. Following..."
            twist = Twist()
            # Control angular para corregir el error
            if abs(error_x) > CENTER_TOLERANCE_X and error_x != -1:
                
                twist.angular.z = -ANGULAR_GAIN * error_x

                if twist.angular.z > MAX_WSPEED:
                    twist.angular.z = MAX_WSPEED
                elif twist.angular.z < -MAX_WSPEED:
                    twist.angular.z = -MAX_WSPEED

                self.last_twist.angular.z = twist.angular.z

            if pixel_depth != -1:
                distance_error = pixel_depth - (TARGET_DISTANCE + self.dynamicDist) # Error respecto a la distancia deseada

                self.last_error = distance_error
                # Control lineal para mantener la distancia
                twist.linear.x = LINEAR_GAIN * distance_error

                if(abs(distance_error) < DISTANCE_ERROR):
                    twist.linear.x = 0
                    if(self.id_flag == "true"):
                        self.cmd_pub.publish(IDENTIFY_CMD)

                if twist.linear.x  > MAX_VSPEED:
                    twist.linear.x  = MAX_VSPEED
                elif twist.linear.x  < -MAX_VSPEED:
                    twist.linear.x  = -MAX_VSPEED

            else: 
                twist.linear.x = self.last_twist.linear.x
                twist.angular.z = self.last_twist.angular.z

            self.last_twist.linear.x = twist.linear.x
            self.last_twist.angular.z = twist.angular.z

            self.cmd_vel_pub.publish(twist)

def main():
    rospy.init_node('FollowPersonNode')
    # Procesar los argumentos de línea de comandos
    
    followNode = FollowPersonNode()

    if followNode.stop_node:
        followNode.log_pub.publish("[INFO] FOLLOW NODE: Node closed")
        followNode.cmd_pub.publish(STOP_FOLLOW_NODE)
        rospy.signal_shutdown("FollowPersonNode_Stop")
        
    rospy.spin()
if __name__ == '__main__':
    main()