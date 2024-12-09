#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2, cv_bridge
import smach_ros
import math
import random
import numpy as np
from time import sleep

from smach import State,StateMachine, Concurrence
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point

""" ******************************************************************************************************
    Definicion de macros y variables globales
"""
TOPIC_VEL = "/cmd_vel_mux/input/navi"
TOPIC_SCAN = '/scan'
TOPIC_COMMAND = '/robot_cmd'
TOPIC_DETECTOR = '/detection_control'
TOPIC_IMAGE = '/image'
TOPIC_PERSONPOSE = '/person_pose'
TOPIC_RGBCAM = '/camera/rgb/image_raw'
TOPIC_DEPTHCAM = '/camera/depth/image_raw'

# MENSAJES
DISABLE_DETECTION_CMD = "start_detection"
ENABLE_DETECTION_CMD = "stop_detection"

START_PATROL_CMD = "start_random_patrol"
STOP_PATROL_CMD = "stop_random_patrol"

START_FOLLOW_CMD = "start_follow_person"
STOP_FOLLOW_CMD = "stop_follow_person"

START_PERIM_CMD = "start_perim_person"
STOP_PERIM_CMD = "stop_perim_person"

START_MOVE_CMD = "start_move_person"
STOP_MOVE_CMD = "stop_move_person"

# ESTADOS
IDLE_ST = "idle_state"
PATROL_ST = "patrol_state"
FOLLOW_ST = "follow_state"
MOVE_ST = "move_state"
PERIM_ST = "perim_state"
HANDLE_ST = "handle_state"

#Nos fijamos en el ángulo a 30 y -30 grados, podéis cambiarlo si quereis
ANG_IZQ = 30*math.pi/180.0
ANG_DER = -ANG_IZQ

# Umbral mínimo de distancia para evitar colisión
UMBRAL_COLISION = 1  # metros
UMBRAL_SINGULARIDAD = 0.5 

UMBRAL_PIXELS = 100

# Velocidades máximas
VEL_MAX_LINEAR = 0.5  # m/s
VEL_MAX_ANGULAR = 2  # rad/s

# Posicion de la base
BASE_POSX = 5
BASE_POSY = 4
BASE_POSZ = 0

# Orientacion de la base (Quaternion)
BASE_ORIX = 0
BASE_ORIY = 0
BASE_ORIZ = 0
BASE_ORIW = 1

# Segundos iniciales 
INIT_SECS = 15

STOP = 0
SEARCH = 1
REACH = 2
                   
# Parámetros de control
CENTER_TOLERANCE_X = 100  # Tolerancia en píxeles para el eje X
TARGET_DISTANCE = 1.5  # Distancia deseada en metros
LINEAR_GAIN = 0.5  # Ganancia para el control de la velocidad lineal
ANGULAR_GAIN = 0.005  # Ganancia para el control de la velocidad angular
MAX_VSPEED = 1
MAX_WSPEED = 0.3

states = [  IDLE_ST,
            PATROL_ST,
            FOLLOW_ST]


""" ******************************************************************************************************
   Clase para el estado de movimiento sin rumbo por el mapa. 
"""   
class RandomPatrol(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != PATROL_ST]
        State.__init__(self, outcomes=[HANDLE_ST,PATROL_ST],output_keys=['output_data'])
        self.patrol = True
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.laser = None
        
    def execute(self, userdata):
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        rate = rospy.Rate(0.5)

        while not rospy.is_shutdown() and self.patrol:
            
            if self.laser is None:
                continue
            
            if np.all(np.isnan(self.laser.ranges)):
                vel_linear = 0
                vel_angular = MAX_WSPEED
                
            else:
                mid_index = int(len(self.laser.ranges) // 2)
                pos_izq = int((ANG_IZQ-self.laser.angle_min)/self.laser.angle_increment)
                pos_der = int((ANG_DER-self.laser.angle_min)/self.laser.angle_increment) 

                # Obtener distancias detectadas por los rayos
                dist_izq = self.laser.ranges[pos_izq]
                dist_der = self.laser.ranges[pos_der]
                dist_delante = self.laser.ranges[mid_index]
                # Cálculo de la media de las distancias
                dist_media = (dist_izq + dist_der) / 2

                # La velocidad lineal se calcula proporcional a la distancia media
                vel_linear = (MAX_VSPEED / 4) * dist_media
                

                if dist_izq < UMBRAL_COLISION or dist_der < UMBRAL_COLISION :
                    # Giro hacia la distancia mayor para evitar el obstaculo más cercano
                    vel_angular = MAX_WSPEED if dist_izq > dist_der else -MAX_WSPEED

                    if abs(dist_izq - dist_der) < UMBRAL_SINGULARIDAD or dist_delante < UMBRAL_SINGULARIDAD:
                        vel_angular = MAX_WSPEED
                else:
                    # La velocidad angular se calcula proporcional a la diferencia de distancias
                    vel_angular = MAX_WSPEED * abs(dist_izq - dist_der)
                    vel_angular = vel_angular if dist_izq > dist_der else -vel_angular  

                # print("Izq", msg.ranges[pos_izq], " Der: ", msg.ranges[pos_der])

            # Mensaje de tipo Twist
            cmd = Twist()
            cmd.linear.x = vel_linear   # Comando de velocidad lineal
            cmd.angular.z = vel_angular # Comando de velocidad angular


            print(cmd)
            if not cmd.linear.x.isnan() and not cmd.angular.z.isnan():
                self.pub.publish(cmd)

            rate.sleep()

        if not self.patrol:
            self.subScan.unregister()
            self.subCmd.unregister()

        if self.cmd == STOP_PATROL_CMD:
            print("Restarting Patrol...")
            return PATROL_ST
        elif self.cmd in self.states:
            print("Ending Patrol...")
            userdata.output_data = self.cmd
            return HANDLE_ST
        
    def cmd_callback(self, msg):
        if msg.data in self.states or msg.data == STOP_PATROL_CMD:
            self.patrol = False
            self.cmd = msg.data

    def laser_callback(self, msg):
        self.laser = msg

""" ******************************************************************************************************
   Clase para el estado de seguimiento de personas.
"""  
class FollowPerson(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != FOLLOW_ST]
        State.__init__(self, outcomes=[HANDLE_ST, FOLLOW_ST],output_keys=['output_data'])
        # Publicador para el tópico de velocidad
        self.cmd_vel_pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.cmd_vision_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        # VARIABLES
        self.last_error = 1
        self.followPerson = True
        self.cmd = None

    def execute(self, userdata):
        
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.subPerson = rospy.Subscriber(TOPIC_PERSONPOSE, Point, self.person_pose_callback)
        self.cmd_vision_pub.publish(ENABLE_DETECTION_CMD)
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown() and self.followPerson:
            rate.sleep()

        if not self.followPerson:
            self.subPerson.unregister()
            self.subCmd.unregister()

        if self.cmd == STOP_FOLLOW_CMD:
            return FOLLOW_ST
        elif self.cmd in self.states:
            userdata.output_data = self.cmd
            return HANDLE_ST

    def cmd_callback(self, msg):
        if msg.data in self.states or msg.data == STOP_FOLLOW_CMD:
            self.followPerson = False
            self.cmd = msg.data

    def person_pose_callback(self, msg):
        """Callback para recibir la posición del pixel desde /person_pose."""

        error_x = msg.x
        pixel_depth = msg.y
        #print("HOLA")
        if error_x == -1 and pixel_depth == -1:   # Person detection has not detected a person
            twist = Twist()
            twist.linear.x = 0

            if self.last_error > 0:
                twist.angular.z = -MAX_WSPEED 
            elif self.last_error < 0:
                twist.angular.z = MAX_WSPEED 
            #rospy.loginfo(f"IDLE MOV: Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
            self.cmd_vel_pub.publish(twist)

        # Verificar que el pixel esté dentro de los límites de la imagen
        else:
            # Obtener la profundidad en el pixel correspondiente
            
            twist = Twist()
            # Control angular para corregir el error
            if abs(error_x) > CENTER_TOLERANCE_X and error_x != -1:
                twist.angular.z = -ANGULAR_GAIN * error_x

                if twist.angular.z > MAX_WSPEED:
                    twist.angular.z = MAX_WSPEED
                elif twist.angular.z < -MAX_WSPEED:
                    twist.angular.z = -MAX_WSPEED
            else:
                twist.angular.z = 0.0

            # Comprobar si la profundidad es válida
            if pixel_depth != -1:
                distance_error = pixel_depth - TARGET_DISTANCE  # Error respecto a la distancia deseada
                self.last_error = distance_error
                # Control lineal para mantener la distancia
                twist.linear.x = LINEAR_GAIN * distance_error
                
                twist.angular.z = twist.angular.z * distance_error
                if twist.linear.x > MAX_VSPEED:
                    twist.linear.x = MAX_VSPEED
                elif twist.linear.x < -MAX_VSPEED:
                    twist.linear.x = -MAX_VSPEED
                #rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
            else: 
                twist.linear.x = 0
                #rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")

            self.cmd_vel_pub.publish(twist)



""" ******************************************************************************************************
   Para esuchar constantemente el TOPIC de CMD
""" 

class IdleWait(State):
    def __init__(self):
        global states
        State.__init__(self, outcomes=states, input_keys=['input_data'])
        self.cmd = None  # Variable para almacenar el último comando recibido
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        self.idleWait = True

    def cmd_callback(self, msg):
        if msg.data in states:
            self.idleWait = False
            self.cmd = msg.data

    def execute(self, userdata):
        rospy.loginfo("IDLE WAIT: Waiting for next state...")
        
        if userdata.input_data in states:
            return userdata.input_data

        # Espera hasta que se reciba un comando válido
        while not rospy.is_shutdown() and self.idleWait:
            rospy.sleep(0.1)  # Espera brevemente antes de verificar nuevamente

        if self.cmd in states:
            return self.cmd

""" ******************************************************************************************************
   Estado manejador
""" 

class MainHandler(State):
    def __init__(self):
        global states
        State.__init__(self, 
                        outcomes=states,
                        input_keys=['next_state'])
        self.cmd = None  # Variable para almacenar el último comando recibido
        #self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String, self.callback)

    def execute(self, userdata):      
        if userdata.next_state in states:
            return userdata.next_state


""" ******************************************************************************************************
   Funcion principal
"""    

def main():
    rospy.init_node("main")

    # Crear un contenedor de concurrencia
    sm = StateMachine(outcomes=['end'])
    sm.userdata.data = None
    with sm:
        StateMachine.add('IdleWait', 
            IdleWait(), 
            transitions={
                IDLE_ST:'IdleWait',
                PATROL_ST:'RandomPatrol',
                FOLLOW_ST:'FollowPerson'},
            remapping={'input_data':'data'})

        StateMachine.add('RandomPatrol', 
            RandomPatrol(), 
            transitions={
                HANDLE_ST:'IdleWait',
                PATROL_ST:'RandomPatrol'},
            remapping={'output_data':'data'})

        StateMachine.add('FollowPerson', 
            FollowPerson(), 
            transitions={
                HANDLE_ST:'IdleWait',
                FOLLOW_ST:'FollowPerson'},
            remapping={'output_data':'data'})
        

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()   


if __name__ == '__main__':
    main()