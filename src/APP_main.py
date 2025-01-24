#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import signal

import rospy
import rosnode
import cv2, cv_bridge
import smach_ros
import math
import random
import numpy as np

import subprocess
import signal

from smach import State, StateMachine
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point

import actionlib
import re

#  rostopic pub /robot_cmd std_msgs/String "start_detection"

from APP_config import TOPIC_LOGS, TOPIC_COMMAND
from APP_config import IDLE_ST, HANDLE_ST, FOLLOW_ST, MOVE_ST, PATROL_ST, QRFINDER_ST, SHUTDOWN_ST, IDENTIFY_ST 
from APP_config import STOP_MOVE_CMD, NODE_SUCCEED, NODE_FAILURE, STOP_FOLLOW_CMD, STOP_FOLLOW_NODE, STOP_MOVE_NODE, STOP_QRFINDER_NODE
from APP_config import MAP_NAME, PACK_NAME, rooms, states 

actual_state = IDLE_ST
""" ******************************************************************************************************
    Clase para el movimiento a un waypoint.
"""  
class MoveState(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != MOVE_ST or state != PATROL_ST]
        State.__init__(self, outcomes=[HANDLE_ST], input_keys=['input_data'], output_keys=['output_data'])
        self.cmd = None
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)  
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)  

        self.move_node_process = None

    def execute(self, userdata):
        global actual_state
        self.cmd_sub = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        
        # Suscriptor al topico /person_pose
        self.cmd = None
        self.log_pub.publish("[INFO] MOVE STATE: Started state")
        rate = rospy.Rate(0.1)
        
        print(userdata.input_data)
        if userdata.input_data in rooms and actual_state == MOVE_ST:
            self.log_pub.publish("[INFO] MOVE STATE: Launching QR waypoint node")
            self.move_node_process = subprocess.Popen(['rosrun', PACK_NAME, 'MR_move_to_qrWaypoint.py', '--place', userdata.input_data])    
        elif re.match(rooms, userdata.input_data) and actual_state == PATROL_ST:
            self.log_pub.publish("[INFO] MOVE STATE: Launching launching patrol area node")
            print("hola")
            self.move_node_process = subprocess.Popen(['rosrun', PACK_NAME, 'MR_patrol_area.py', '--place', userdata.input_data])
        elif userdata.input_data == "route" and actual_state == PATROL_ST: 
            self.log_pub.publish("[INFO] MOVE STATE: Launching launching patrol route node")
            self.move_node_process = subprocess.Popen(['rosrun', PACK_NAME, 'MR_patrol_route.py'])
        elif actual_state == MOVE_ST:
            values = userdata.input_data.split(',')
            x, y, w = map(float, values)
            self.log_pub.publish("[INFO] MOVE STATE: Launching Move to point node")
            self.move_node_process = subprocess.Popen(['rosrun', PACK_NAME, 'MR_move_to_point.py', '--x', str(x), '--y', str(y), '--w', str(w)])
        while not rospy.is_shutdown():
            rate.sleep()

            if self.cmd is not None:
                if self.cmd == STOP_MOVE_NODE:
                    break
                elif self.cmd in self.states:
                    userdata.output_data = self.cmd  
                    self.cmd_pub.publish(STOP_MOVE_CMD)    
                
        self.log_pub.publish("[INFO] MOVE STATE: Stopped state")

        self.log_pub.publish(f"[EVENT]: MOVE STATE -> IDLE STATE")
        return HANDLE_ST
            
    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)
        
    def cmd_callback(self, msg):

        if msg.data in self.states:
            rospy.loginfo("[INFO] MOVE STATE: Movement aborted by command.")
            self.log_pub.publish("[INFO] MOVE STATE: Movement aborted by command.")
            
        self.cmd = msg.data
""" ******************************************************************************************************
   Clase para el estado de seguimiento de personas.
"""  
class FollowPersonState(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != FOLLOW_ST]
        State.__init__(self, outcomes=[HANDLE_ST], input_keys=['input_data'], output_keys=['output_data'])
        # Publicador para el tópico de velocidad
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        
        self.log_msg = "[INFO] FOLLOW STATE: Waiting..."
        # VARIABLES
        self.followPerson = True
        self.cmd = None
        self.follow_person_process = None

    def execute(self, userdata):
        global actual_state
        self.cmd_sub = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        #self.timer = rospy.Timer(rospy.Duration(5), self.publish_message)
        
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.log_pub.publish("[INFO] FOLLOW STATE: Started state")
        rate = rospy.Rate(0.1)

        if actual_state == FOLLOW_ST:
            self.follow_person_process = subprocess.Popen(['rosrun', PACK_NAME, 'MR_follow_person.py', '--identify', "false"])
        elif actual_state == IDENTIFY_ST:
            self.follow_person_process = subprocess.Popen(['rosrun', PACK_NAME, 'MR_follow_person.py', '--identify', "true"])

        while not rospy.is_shutdown() and self.followPerson:
            rate.sleep()

            if self.cmd is not None:
                if self.cmd == STOP_FOLLOW_NODE:
                    break
                elif self.cmd in self.states:
                    userdata.output_data = self.cmd  
                    self.cmd_pub.publish(STOP_FOLLOW_CMD)
                    break
                
        self.log_pub.publish("[INFO] FOLLOW STATE: Stopped state")     
        self.log_pub.publish(f"[EVENT]: FOLLOW STATE -> IDLE STATE")
        
        return HANDLE_ST           
            
    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)
        
    def cmd_callback(self, msg):
        self.cmd = msg.data

""" ******************************************************************************************************
   Clase para el estado de encontrar QRs por el entorno.
"""  
class QRFinderState(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != QRFINDER_ST]
        State.__init__(self, outcomes=[HANDLE_ST], input_keys=['input_data'], output_keys=['output_data'])
        # Publicador para el tópico de velocidad
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        self.qrmove_node_process = None
        self.log_msg = "[INFO] QR FINDER STATE: Waiting..."
        # VARIABLES
        self.cmd = None

    def execute(self, userdata):
        
        self.cmd_sub = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        self.timer = rospy.Timer(rospy.Duration(5), self.publish_message)
        
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.log_pub.publish("[INFO] QR FINDER STATE: Started state")
        rate = rospy.Rate(0.1)

        self.qrmove_node_process = subprocess.Popen(['rosrun', PACK_NAME, 'RV_QR_finder.py'])

        while not rospy.is_shutdown():
            rate.sleep()
            
            if self.cmd is not None:
                if self.cmd == STOP_QRFINDER_NODE:
                    break
                elif self.cmd in self.states:
                    userdata.output_data = self.cmd  
                    self.cmd_pub.publish(STOP_MOVE_CMD)
                    break
                
        self.log_pub.publish("[INFO] QR FINDER STATE: Stopped state")
        self.log_pub.publish(f"[EVENT]: QR FINDER STATE -> IDLE STATE")
        self.timer.shutdown() 
        return HANDLE_ST
    
    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)
        
    def cmd_callback(self, msg):
        self.cmd = msg.data
        
""" ******************************************************************************************************
   Para esuchar constantemente el TOPIC de CMD
""" 
class IdleWait(State):
    def __init__(self):
        global states
        
        State.__init__(self, outcomes=states, input_keys=['input_data'], output_keys=['output_data'])
        self.cmd = None  # Variable para almacenar el último comando recibido
        self.cmd_sub = rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)

        self.idleWait = True
        self.arg = None
        self.log_msg = "[INFO] IDLE STATE: Waiting for next state..."

    def execute(self, userdata):
        global actual_state
        rospy.loginfo("[INFO] IDLE STATE: Waiting for next state...")  
        self.timer = rospy.Timer(rospy.Duration(10), self.publish_message)

        # Espera hasta que se reciba un comando válido
        while not rospy.is_shutdown() and self.idleWait:
            rospy.sleep(0.1)  # Espera brevemente antes de verificar nuevamente

        if userdata.input_data in states:
            cmd = userdata.input_data
            userdata.output_data = cmd
            actual_state = cmd
            self.log_pub.publish(f"[EVENT]: IDLE STATE -> {cmd}")
            self.timer.shutdown()
            return userdata.input_data
        
        if self.cmd in states:
            userdata.output_data = self.arg
            self.log_pub.publish(f"[EVENT]: IDLE STATE -> {self.cmd}")
            self.timer.shutdown()

            cmd = self.cmd
            actual_state = cmd
            self.cmd = None   
            return cmd

        self.idleWait = True
        self.timer.shutdown()
        return IDLE_ST
                    
    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)
        
    def cmd_callback(self, msg):
        if ":" in msg.data: 
            cmd, self.arg = msg.data.split(":")
        else:
            cmd = msg.data

        if cmd in states:
            self.idleWait = False
            self.cmd = cmd

""" ******************************************************************************************************
   Funcion para parar todos los nodos
"""    
def stop_all_nodes():
    rospy.loginfo("Recuperando lista de nodos...")
    try:
        # Obtener todos los nodos en ejecución
        nodes = rosnode.get_node_names()
        log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        # Separar el nodo "main" del resto
        main_node = None
        other_nodes = []
        for node in nodes:
            if node == "/main":  # Nombre exacto del nodo principal
                main_node = node

            elif node != "/bash_interface" or node != "/visual_interface" :
                other_nodes.append(node)
        
        log_pub.publish(f"[EVENT]: CLOSING APP")

        # Apagar todos los nodos excepto "main"
        for node in other_nodes:
            try:
                rospy.loginfo(f"Apagando nodo: {node}")
                os.system(f"rosnode kill {node}")
            except Exception as e:
                rospy.logwarn(f"Error al intentar apagar {node}: {e}")

        # Finalmente, apagar el nodo "main"
        if main_node:
            try:
                rospy.loginfo(f"Apagando el nodo principal: {main_node}")
                os.system(f"rosnode kill {main_node}")
                rospy.signal_shutdown("Apagando")
                
            except Exception as e:
                rospy.logwarn(f"Error al intentar apagar {main_node}: {e}")

    except Exception as e:
        rospy.logerr(f"Error al recuperar los nodos: {e}")
        
""" ******************************************************************************************************
   Funcion para parar todos los nodos
"""        
def kill_run_launch():
    try:
        # Ejecuta el comando para buscar procesos relacionados con run.launch
        result = subprocess.check_output(
            "ps aux | grep '[r]oslaunch.*run.launch'",
            shell=True,
            text=True
        )
        
        # Filtrar el PID del resultado
        lines = result.strip().split("\n")
        for line in lines:
            parts = line.split()
            pid = int(parts[1])  # El PID es la segunda columna
            print(f"Encontrado proceso run.launch con PID: {pid}")
            
            # Termina el proceso
            os.kill(pid, signal.SIGTERM)  # O usa signal.SIGKILL para forzar
            print(f"Proceso con PID {pid} terminado correctamente.")
            
    except subprocess.CalledProcessError:
        print("No se encontró ningún proceso relacionado con run.launch.")
    except Exception as e:
        print(f"Error al intentar matar el proceso: {e}")
        
        
""" ******************************************************************************************************
   Funcion principal
"""
def main():
    rospy.init_node("main")
        
    log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
    # Crea una maquina de estados mediante SMACH
    sm = StateMachine(outcomes=['end'])
    
    log_pub.publish("[EVENT]: STARTING APP")
    log_pub.publish(f"[EVENT]: USING MAP NAME {MAP_NAME}")
    print(f"[EVENT]: USING MAP NAME {MAP_NAME}")
    
    sm.userdata.data = None

    with sm:
        # Estado IdleWait, el robot espera a recibir un comando               
        StateMachine.add('IdleWait', 
            IdleWait(), 
            transitions={
                IDLE_ST:'IdleWait',
                FOLLOW_ST:'FollowPerson',
                IDENTIFY_ST:'FollowPerson',
                MOVE_ST:'MoveState',
                PATROL_ST:'MoveState',
                QRFINDER_ST:'QRFinder',
                SHUTDOWN_ST:'end'},
            remapping={'input_data':'data',
                       'output_data':'data'})

        # Estado FollowPerson, el robot sigue a una persona
        StateMachine.add('FollowPerson', 
            FollowPersonState(), 
            transitions={
                HANDLE_ST:'IdleWait'},
            remapping={'input_data':'data',
                       'output_data':'data'})
        
        # Estado MoveState, el robot se mueve hacia una posicion
        StateMachine.add('MoveState', 
            MoveState(), 
            transitions={
                HANDLE_ST:'IdleWait'},
            remapping={'input_data': 'data'})
        
        # Estado QRFinder, el robot busca de manera aleatoria por el mapa QRs
        StateMachine.add('QRFinder', 
            QRFinderState(), 
            transitions={
                HANDLE_ST:'IdleWait'},
            remapping={'input_data':'data',
                       'output_data':'data'})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    res = sm.execute()

    if res == 'end':
        stop_all_nodes()
        kill_run_launch()
        rospy.signal_shutdown('exit')

    rospy.spin()   


if __name__ == '__main__':
    main()