import rospy
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import cv2
import numpy as np
import mediapipe as mp

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import re

from APP_main import TOPIC_COMMAND, TOPIC_LOGS, TOPIC_VEL, TOPIC_PERSONPOSE, MOVE_CLOSER_CMD, MOVE_AWAY_CMD, RESET_DIST_CMD, IDENTIFY_CMD 
from APP_main import START_DETECTION_CMD, STOP_DETECTION_CMD, START_FOLLOW_CMD, STOP_FOLLOW_CMD, NODE_SUCCEED, NODE_FAILURE
from APP_main import MAX_VSPEED, MAX_WSPEED, ANGULAR_GAIN, LINEAR_GAIN, TARGET_DISTANCE, DISTANCE_ERROR, CENTER_TOLERANCE_X


class FollowPerson():
    def __init__(self):

        # Publicador para el tópico de velocidad
        self.cmd_vel_pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.cmd_vision_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        
        self.log_msg = "[INFO] FOLLOW STATE: Waiting..."
        # VARIABLES
        self.last_error = 1
        self.followPerson = True
        self.cmd = None
        self.dynamicDist = 0
        self.last_twist = Twist()
        self.last_twist.angular.z = 0
        self.last_twist.linear.x = 0
        
    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)

    def execute(self):
        
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        self.timer = rospy.Timer(rospy.Duration(5), self.publish_message)
        
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.subPerson = rospy.Subscriber(TOPIC_PERSONPOSE, Point, self.person_pose_callback)
        self.cmd_vision_pub.publish(START_DETECTION_CMD)
        self.log_pub.publish("[INFO] FOLLOW STATE: Started state")
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown() and self.followPerson:
            rate.sleep()

            if self.cmd == STOP_FOLLOW_CMD:
                self.log_pub.publish("[INFO] FOLLOW STATE: Stopped state")
                self.subPerson.unregister()
                self.subCmd.unregister()

                self.dynamicDist = 0
                self.last_error = 0
                self.followPerson = False     
                
                self.cmd_vision_pub.publish(STOP_DETECTION_CMD)
                self.timer.shutdown()
                self.cmd_pub.publish(NODE_SUCCEED)

    def cmd_callback(self, msg):
   
        if msg.data == MOVE_CLOSER_CMD:
            self.dynamicDist = self.dynamicDist - 0.5
        elif msg.data == MOVE_AWAY_CMD:
            self.dynamicDist = self.dynamicDist + 0.5
        elif msg.data == RESET_DIST_CMD:
            self.dynamicDist = 0
            
        if msg.data == START_FOLLOW_CMD:
            self.execute()

        self.cmd = msg.data

    def person_pose_callback(self, msg):
        """Callback para recibir la posición del pixel desde /person_pose."""
        if self.cmd == STOP_FOLLOW_CMD:
            self.subPerson.unregister()  
        
        error_x = msg.x
        pixel_depth = msg.y
        
        if error_x == -1 and pixel_depth == -1:   # Person detection has not detected a person
            twist = Twist()
            twist.linear.x = 0

            if self.last_error > 0:
                twist.angular.z = -MAX_WSPEED 
            else:
                twist.angular.z = MAX_WSPEED 
            
            self.log_msg = "[INFO] FOLLOW STATE: Searching for person..."
            self.cmd_vel_pub.publish(twist)

        # SI HAY UNA PERSONA EN LA IMAGEN
        else:
            # Obtener la profundidad en el pixel correspondiente
            self.log_msg = "[INFO] FOLLOW STATE: Person found. Following..."
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

