#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2, cv_bridge
import smach_ros
import math
import random
import numpy as np
from time import sleep

from smach import State,StateMachine
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/base_scan'
TOPIC_COLOR = '/color_detected'
TOPIC_DETECTOR = '/detection_control'
TOPIC_IMAGE = '/image'

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
                   
""" ******************************************************************************************************
   Clase para el estado inicial que realiza una rotacion del robot durante un numero aleatorio de segundos
   para aleatorizar el recorrido del mapa.
"""
class InitialMovement(State):
    def __init__(self):
        State.__init__(self, outcomes=['init_done'])
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        
        self.initial_turn_duration = random.randint(5, INIT_SECS)   # Obtenemos un numero aleatorio de segundos en los que el robot permanecerá rotando
        self.init_done = False

        # Configurar el temporizador
        self.duration = rospy.Duration(self.initial_turn_duration)  # Duración total del timer
        self.start_time = rospy.Time.now()                          # Inicio del timer
        rospy.loginfo(f"Inicializando en T: {self.duration.to_sec()} segundos T0 = {self.start_time.to_sec()}")
        # Crear el temporizador que llama a `self.timer_callback` cada 1 segundo
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def execute(self, userdata):    

        rate = rospy.Rate(10)

        while not self.init_done:   # Se comprueba si se ha cumplido la inicialización
            rate.sleep()
             
        return "init_done"
    
    def timer_callback(self, event):
        elapsed_time = rospy.Time.now() - self.start_time   # Se calcula el tiempo transcurrido desde que se inició el estado
        rospy.loginfo(f"Tiempo transcurrido: {elapsed_time.to_sec()} segundos")

        cmd = Twist()
        cmd.linear.x = 0.0                              # Comando de velocidad lineal 
        cmd.angular.z = -VEL_MAX_ANGULAR                # Comando de velocidad angular
        self.pub.publish(cmd)

        # Detener el temporizador después de que pase la duración total
        if elapsed_time >= self.duration:
            rospy.loginfo("Duración completada. Deteniendo temporizador.")
            publish_detection_control(True)                  # Publicacion del flag de activacion del detector de color en el topic /detection_control
            self.init_done = True                            # Salimos al siguiente estado
            self.timer.shutdown()

""" ******************************************************************************************************
   Clase para el estado de movimiento sin rumbo por el mapa y deteccion de color rojo. 
"""   
class WanderAndDetect(State):
    def __init__(self):
        State.__init__(self, outcomes=['color_detected'])
        self.color_detected = False
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
    
    def execute(self, userdata):
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)
        self.subColor = rospy.Subscriber(TOPIC_COLOR, Int32 , self.color_detected_callback)
        rate = rospy.Rate(1)

        while not self.color_detected:
            rate.sleep()

        if self.color_detected:
            self.subScan.unregister()
            self.subColor.unregister()

        publish_detection_control(False)    # Desactivacion de la deteccion de color (desuscripcion del topic /image)
        return "color_detected"
        
    
    def laser_callback(self, msg):

        
        mid_index = int(len(msg.ranges) // 2)
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 

        # Obtener distancias detectadas por los rayos
        dist_izq = msg.ranges[pos_izq]
        dist_der = msg.ranges[pos_der]
        dist_delante = msg.ranges[mid_index]
        # Cálculo de la media de las distancias
        dist_media = (dist_izq + dist_der) / 2

        # La velocidad lineal se calcula proporcional a la distancia media
        vel_linear = (VEL_MAX_LINEAR / 4) * dist_media
        

        if dist_izq < UMBRAL_COLISION or dist_der < UMBRAL_COLISION :

            # Giro hacia la distancia mayor para evitar el obstaculo más cercano
            vel_angular = VEL_MAX_ANGULAR if dist_izq > dist_der else -VEL_MAX_ANGULAR

            if abs(dist_izq - dist_der) < UMBRAL_SINGULARIDAD or dist_delante < UMBRAL_SINGULARIDAD:
                vel_angular = VEL_MAX_ANGULAR

        else:

            # La velocidad angular se calcula proporcional a la diferencia de distancias
            vel_angular = VEL_MAX_ANGULAR * abs(dist_izq - dist_der)
            vel_angular = vel_angular if dist_izq > dist_der else -vel_angular  

        # print("Izq", msg.ranges[pos_izq], " Der: ", msg.ranges[pos_der])

        # Mensaje de tipo Twist
        cmd = Twist()
        cmd.linear.x = vel_linear   # Comando de velocidad lineal
        cmd.angular.z = vel_angular # Comando de velocidad angular
        self.pub.publish(cmd)

    def color_detected_callback(self, msg):
        rospy.loginfo('Objeto rojo detectado. Volviendo a base.')
        
        self.color_detected = True


""" ******************************************************************************************************
   Funcion de control para el detector de color
"""   
def publish_detection_control(state):
    
    #rospy.init_node('detection_control_publisher', anonymous=True)
    pub = rospy.Publisher(TOPIC_DETECTOR, Bool, queue_size=10)
    
    rospy.sleep(1)    
    # Crear y publicar el mensaje
    msg = Bool()
    msg.data = state
    pub.publish(msg)  
        
if __name__ == '__main__':

    rospy.init_node("practica3")

    # Inicializacion de la posicion de la base
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = BASE_POSX
    goal_pose.target_pose.pose.position.y = BASE_POSY
    goal_pose.target_pose.pose.position.z = BASE_POSZ
    goal_pose.target_pose.pose.orientation.x = BASE_ORIX
    goal_pose.target_pose.pose.orientation.y = BASE_ORIY
    goal_pose.target_pose.pose.orientation.z = BASE_ORIZ
    goal_pose.target_pose.pose.orientation.w = BASE_ORIW

    # Creacion de la maquina de estados
    sm = StateMachine(outcomes=['end'])
    with sm:
       
        # Estado inicial rotacion aleatoria
        StateMachine.add('InitialMovement', 
            InitialMovement(), 
            transitions={
               'init_done':'WanderAndDetect'})

        # Estado movimiento sin rumbo y deteccion de color
        StateMachine.add('WanderAndDetect', 
            WanderAndDetect(), 
            transitions={
               'color_detected':'MoveToBase'})
        
        # Estado regresar a base
        StateMachine.add('MoveToBase',
            SimpleActionState('move_base',MoveBaseAction,goal=goal_pose),
            transitions={'succeeded':'end', 'aborted':'end', 'preempted':'end'})
        
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()   
