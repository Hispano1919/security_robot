#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
""" ******************************************************************************************************
    Definicion de macros y variables globales
"""
PACK_NAME = "security_robot"
TOPIC_VEL = "/cmd_vel_mux/input/navi"
TOPIC_SCAN = '/scan'
TOPIC_AMCLPOS = '/amcl_pose'
TOPIC_COMMAND = '/robot_cmd'
TOPIC_DETECTOR = '/detection_control'
TOPIC_IMAGE = '/image'
TOPIC_PERSONPOSE = '/person_pose'
TOPIC_RGBCAM = '/camera/rgb/image_raw'
TOPIC_DEPTHCAM = '/camera/depth/image_raw'
TOPIC_LOGS = '/robot_logs'
TOPIC_PRIMG = '/processed_image'

# MENSAJES
MOVE_CLOSER_CMD = "move_closer"
MOVE_AWAY_CMD = "move_away"
RESET_DIST_CMD = "reset_dist"

START_DETECTION_CMD = "start_detection"
STOP_DETECTION_CMD = "stop_detection"
STOP_QRFINDER_NODE = "stop_qrfinder_node"

START_FOLLOW_CMD = "start_follow_person"
STOP_FOLLOW_CMD = "stop_follow_person"
STOP_FOLLOW_NODE = "stop_follow_node"

START_MOVE_CMD = "start_move_robot"
STOP_MOVE_CMD = "stop_move_robot"
STOP_MOVE_NODE = "stop_move_node"

START_VOICE_CMD = "start_voice_control"
STOP_VOICE_CMD = "stop_voice_control"

NODE_SUCCEED = "node_succeed"
NODE_FAILURE = "node_failure"

SHUTDOWN_CMD = "goodbye"
IDENTIFY_CMD = "identify_command"

# ESTADOS
IDLE_ST = "idle_state"
FOLLOW_ST = "follow_state"
IDENTIFY_ST = "identify_state"
MOVE_ST = "move_state"
HANDLE_ST = "handle_state"
SHUTDOWN_ST = "shutdown_state"
BASE_ST = "base_state"
PATROL_ST = "patrol_state"
QRFINDER_ST = "qrfinder_state"


# PATH
WAYPOINT_PATH = "/home/asahel/ROS_WS/src/security_robot/output_files/default.qrlog"
MAP_NAME = "casa4"



# Parámetros de control
CENTER_TOLERANCE_X = 50  # Tolerancia en píxeles para el eje X
TARGET_DISTANCE = 1.5  # Distancia deseada en metros
DISTANCE_ERROR = 0.1
LINEAR_GAIN = 0.5  # Ganancia para el control de la velocidad lineal
ANGULAR_GAIN = 0.001  # Ganancia para el control de la velocidad angular
MAX_VSPEED = 0.5
MAX_WSPEED = 0.5

states = [IDLE_ST, FOLLOW_ST, MOVE_ST, SHUTDOWN_ST, PATROL_ST, QRFINDER_ST, IDENTIFY_ST]
rooms = r"^(.*)\b(cocina|wc|salon|habitacion|estacion|base|area(_\d+)?)\b$"

# Definir los waypoints a los que el robot debe moverse
def_waypoints = [
    ['habitacion', (-0.3, 4.4), (0.0, 0.0, 0.0, 1.0)],  # Nombre, posición, orientación
    ['estacion', (-2.06, 5.82), (0.0, 0.0, 0.0, 1.0)],  # Ejemplo con otro punto
    ['wc', (-3.6, 4.6), (0.0, 0.0, 0, 1.0)],
    ['salon', (-3.6, 0.7), (0.0, 0.0, 0, 1.0)],
    ['cocina', (0.0, 1.6), (0.0, 0.0, 0, 1.0)],
]

""" ******************************************************************************************************
    Funcion para actualizar la variable global MAP_NAME
"""
def update_map_name(map_name):
    global MAP_NAME  # Declarar MAP_NAME como global
    MAP_NAME = map_name
    print(f"MAP_NAME updated to: {MAP_NAME}")
    
""" ******************************************************************************************************
    Funcion principal
"""
def main():
    
    # Simulación de la lógica principal del script
    print("Running the robot control script...")
    print(f"Using MAP_NAME: {MAP_NAME}")

if __name__ == "__main__":
    main()