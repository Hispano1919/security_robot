#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function  # Asegura que print sea una función en Python 2.7
import tkinter as tk
import cv2
import numpy as np
import rospy
import actionlib
import yaml
import math
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from PIL import Image as PILImage, ImageTk  # Usamos PILImage para evitar conflictos con sensor_msgs.Image
import csv
import subprocess  # Para llamar al script
#import patrullaje  # Importar el módulo directamente
from cv_bridge import CvBridge
from sensor_msgs.msg import Image  # Asegurarse de que la clase Image de ROS se importa correctamente
from std_msgs.msg import String

from APP_config import rooms, TOPIC_COMMAND, TOPIC_LOGS, FOLLOW_ST, SHUTDOWN_ST, MOVE_ST, PATROL_ST, MAP_NAME, TOPIC_RGBCAM
from APP_config import STOP_FOLLOW_CMD, START_DETECTION_CMD, STOP_DETECTION_CMD, START_VOICE_CMD, STOP_VOICE_CMD

class MapaApp:
    def __init__(self, master, ruta_mapa, ruta_yaml, ruta_mapa_coloreado, ruta_csv):
        self.master = master
        self.ruta_mapa = ruta_mapa
        self.ruta_yaml = ruta_yaml
        self.ruta_mapa_coloreado = ruta_mapa_coloreado
        self.ruta_csv = ruta_csv

        # Inicializar CvBridge para convertir mensajes ROS a imágenes OpenCV
        self.bridge = CvBridge()

        # Crear un Canvas para mostrar la cámara
        self.canvas_camera = tk.Canvas(self.master, width=640, height=480)
        self.canvas_camera.grid(row=0, column=1, padx=10, pady=10, sticky="ne")

        # Suscribirse al topic de la cámara
        self.image_sub = rospy.Subscriber(TOPIC_RGBCAM, Image, self.callback_camera)

        # Cargar mapas y parámetros
        self.mapa = cv2.imread(ruta_mapa)
        self.mapa_binario = cv2.cvtColor(self.mapa, cv2.COLOR_BGR2GRAY)
        self.resolucion, self.origen = self.leer_parametros_mapa(ruta_yaml)
        self.mapa_coloreado = cv2.imread(ruta_mapa_coloreado)
        self.colores_y_rutas = self.cargar_csv_colores(ruta_csv)

        self.coordenada_inicio = None  # Guardamos la primera selección (ubicación)
        self.mapa_original = self.mapa.copy()  # Copia del mapa original para restablecerlo

        # Convertir la imagen de OpenCV (BGR) a formato Tkinter (RGB)
        self.mapa_tk = ImageTk.PhotoImage(image=PILImage.fromarray(cv2.cvtColor(self.mapa, cv2.COLOR_BGR2RGB)))
        self.mapa_coloreado_tk = ImageTk.PhotoImage(image=PILImage.fromarray(cv2.cvtColor(self.mapa_coloreado, cv2.COLOR_BGR2RGB)))

        # Crear el Canvas de Tkinter para mostrar el mapa
        self.canvas_mapa = tk.Canvas(self.master, width=self.mapa_tk.width(), height=self.mapa_tk.height())
        self.canvas_mapa.create_image(0, 0, anchor=tk.NW, image=self.mapa_tk)
        self.canvas_mapa.grid(row=0, column=0, padx=10, pady=10, sticky="nw")  # Coloca en la fila 1, columna 0

        # Crear el Canvas de Tkinter para mostrar el mapa coloreado
        self.canvas_mapa_coloreado = tk.Canvas(self.master, width=self.mapa_coloreado_tk.width(), height=self.mapa_coloreado_tk.height())
        self.canvas_mapa_coloreado.create_image(0, 0, anchor=tk.NW, image=self.mapa_coloreado_tk)
        self.canvas_mapa_coloreado.grid(row=1, column=0, padx=10, pady=10, sticky="nw")  # Coloca en la fila 1, columna 1

        # Crear un Frame para contener los botones
        self.frame_botones = tk.Frame(self.master, bg="lightgrey", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.frame_botones.grid(row=1, column=1, sticky="nsew", padx=10, pady=10, columnspan=1)

        # Hacer que el grid se expanda para ocupar el espacio disponible
        self.master.grid_rowconfigure(1, weight=1)  # Esto hace que la fila 1 (donde está el frame_botones) se expanda
        self.master.grid_columnconfigure(1, weight=1)  # Esto hace que la columna 1 (donde está el frame_botones) se expanda

        # Añadir botones al Frame
        self.boton1 = tk.Button(self.frame_botones, text="Stop Robot", command=self.funcion_boton1, width=15, bg="#007BFF", fg="white")
        self.boton2 = tk.Button(self.frame_botones, text="Go home", command=self.funcion_boton2, width=15, bg="#28A745", fg="white")
        self.boton3 = tk.Button(self.frame_botones, text="Centroides", command=self.funcion_boton3, width=15, bg="#DC3545", fg="white")

        # Organizar los botones verticalmente en el Frame
        self.boton1.pack(side="top", pady=5)
        self.boton2.pack(side="top", pady=5)
        self.boton3.pack(side="top", pady=5)

        # Integrar la consola de comandos
        self.command_frame = tk.Frame(self.master, bg="lightgrey", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.command_frame.grid(row=1, column=2, columnspan=2, padx=10, pady=10, sticky="nsew")

        # Crear widgets para la consola de comandos
        self.console = tk.Text(self.command_frame, height=10, width=60)
        self.console.pack()

        self.command_entry = tk.Entry(self.command_frame, width=60)
        self.command_entry.pack()

        self.send_button = tk.Button(self.command_frame, text="Enviar", command=self.send_command)
        self.send_button.pack()

        # Crear los Publishers y Subscriber de ROS
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=1)
        rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)


        # Configurar los eventos de clic
        self.canvas_mapa.bind("<Button-1>", self.click_event_mapa)
        self.canvas_mapa_coloreado.bind("<Button-1>", self.detectar_color_pixel)

    def cmd_callback(self, msg):
        if msg.data == SHUTDOWN_ST:
            self.console.insert(tk.END, "Apagando robot...\n")
            self.master.quit()

    def send_command(self):
        command = self.command_entry.get()
        self.console.insert(tk.END, "Comando recibido: {}\n".format(command))
        self.handle_command(command)

    def handle_command(self, command):
        # Procesa los comandos ingresados
        command = command.lower()

        if "sigue" in command or "sigueme" in command:
            self.console.insert(tk.END, "Siguiendo...\n")
            self.cmd_pub.publish(FOLLOW_ST)
        elif "quieto" in command or "quedate" in command:
            self.console.insert(tk.END, "Quedándome quieto...\n")
            self.cmd_pub.publish(STOP_FOLLOW_CMD)
        elif "muevete" in command or "ve" in command:
            self.console.insert(tk.END, "Comando recibido: {}\n".format(command))
            self.cmd_pub.publish(MOVE_ST + ':' + command)
        elif "camara" in command:
            if "enciende" in command:
                self.console.insert(tk.END, "Cámara encendida.\n")
                self.cmd_pub.publish(START_DETECTION_CMD)
            elif "apaga" in command:
                self.console.insert(tk.END, "Cámara apagada.\n")
                self.cmd_pub.publish(STOP_DETECTION_CMD)
        elif "voz" in command:
            if "enciende" in command:
                self.console.insert(tk.END, "Detección de voz encendida.\n")
                self.cmd_pub.publish(START_VOICE_CMD)
            elif "apaga" in command:
                self.console.insert(tk.END, "Detección de voz apagada.\n")
                self.cmd_pub.publish(STOP_VOICE_CMD)
        elif "salir" in command or "adios" in command:
            self.console.insert(tk.END, "Apagando robot...\n")
            self.cmd_pub.publish(SHUTDOWN_ST)
            self.master.quit()

    def funcion_boton1(self):
        print("Botón 1 presionado.")

    def funcion_boton2(self):
        print("Botón 2 presionado.")

    def funcion_boton3(self):
        print("Botón 3 presionado.")

    def callback_camera(self, msg):
        """Recibir imágenes de la cámara y actualizar la interfaz."""
        try:
            # Convertir el mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convertir a formato que Tkinter pueda mostrar
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            image_pil = PILImage.fromarray(cv_image_rgb)
            image_tk = ImageTk.PhotoImage(image=image_pil)

            # Actualizar el canvas con la nueva imagen en un hilo separado
            self.master.after(0, self.actualizar_imagen, image_tk)

        except Exception as e:
            print("Error al recibir imagen de la cámara: ", e)

    def actualizar_imagen(self, image_tk):
        """Actualizar la imagen de la cámara en el canvas."""
        self.canvas_camera.create_image(0, 0, anchor=tk.NW, image=image_tk)
        self.canvas_camera.image = image_tk

    def cargar_mapa_coloreado(self, ruta_mapa_coloreado):
        """Cargar el mapa coloreado para interactuar con él."""
        mapa = cv2.imread(ruta_mapa_coloreado)
        if mapa is None:
            raise IOError("No se pudo cargar el mapa coloreado en {0}".format(ruta_mapa_coloreado))
        return mapa

    def cargar_csv_colores(self, ruta_csv):
        """
        Cargar el CSV que contiene los colores y las rutas a los archivos .pgm de las áreas segmentadas.
        """
        colores_y_rutas = []
        with open(ruta_csv, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                # Ajustar el formato del color para que sea una tupla de enteros
                color = tuple(map(int, row['Color (R,G,B)'].strip('[]').split(',')))
                ruta_pgm = row['Ruta .pgm']
                colores_y_rutas.append((color, ruta_pgm))
        return colores_y_rutas

    def encontrar_area_por_color(self, color, colores_y_rutas):
        """
        Buscar el archivo .pgm correspondiente al color seleccionado.
        """
        for area_color, ruta_pgm in colores_y_rutas:
            if area_color == color:
                return ruta_pgm
        return None

    def leer_parametros_mapa(self, ruta_yaml):
        """Leer resolución y origen del mapa desde el archivo YAML."""
        with open(ruta_yaml, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        resolucion = yaml_data['resolution']
        origen = yaml_data['origin']  # Origen en coordenadas del mundo real (x, y, theta)
        return resolucion, (origen[0], origen[1])

    def convertir_a_coordenadas(self, px, py):
        """
        Convertir las coordenadas de un píxel en el mapa a coordenadas reales en el marco 'map'.
        """
        x = px * self.resolucion + self.origen[0]
        y = self.origen[1] + (self.mapa_binario.shape[0] - py) * self.resolucion
        return x, y

    def calcular_orientacion(self, x1, y1, x2, y2):
        """
        Calcular la orientación (ángulo theta) entre dos puntos (x1, y1) y (x2, y2).
        """
        delta_x = x2 - x1
        delta_y = y2 - y1
        return math.atan2(delta_y, delta_x)

    def mover_a_goal(self, x, y, theta):
        """Enviar un goal al robot para moverse a una ubicación específica con una orientación."""
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(theta / 2)
        goal.target_pose.pose.orientation.w = math.cos(theta / 2)

        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def ejecutar_movimiento(self, coord_x, coord_y, theta):
        """Ejecutar movimiento del robot en un hilo separado."""
        thread = threading.Thread(target=self.mover_a_goal, args=(coord_x, coord_y, theta))
        thread.start()

    def ejecutar_patrullaje(self, ruta_pgm):
        """Ejecutar el patrullaje en un hilo separado."""
        thread = threading.Thread(target=self.patrullar, args=(ruta_pgm,))
        thread.start()

    def patrullar(self, ruta_pgm):
        """Función que llama al módulo de patrullaje y realiza el patrullaje."""

    def click_event_mapa(self, event):
        """Evento para manejar clics en el mapa y mover el robot."""
        x = event.x
        y = event.y

        if self.mapa_binario[y, x] >= 240:
            if self.coordenada_inicio is None:
                # Primer clic: seleccionar ubicación
                self.coordenada_inicio = (x, y)
                coord_x, coord_y = self.convertir_a_coordenadas(x, y)
                print("Ubicación seleccionada (en metros): x = {0}, y = {1}".format(coord_x, coord_y))
                cv2.circle(self.mapa, (x, y), 5, (0, 255, 0), -1)  # Marcar el primer punto
            else:
                # Segundo clic: seleccionar orientación
                coord_x_inicio, coord_y_inicio = self.convertir_a_coordenadas(self.coordenada_inicio[0], self.coordenada_inicio[1])
                coord_x_final, coord_y_final = self.convertir_a_coordenadas(x, y)

                # Calcular la orientación en radianes
                theta = self.calcular_orientacion(coord_x_inicio, coord_y_inicio, coord_x_final, coord_y_final)
                print("Orientación seleccionada (en radianes): {0}".format(theta))

                # Ejecutar el movimiento del robot en un hilo separado
                self.ejecutar_movimiento(coord_x_inicio, coord_y_inicio, theta)

                # Restablecer la primera selección
                self.coordenada_inicio = None

    def detectar_color_pixel(self, event):
        """
        Manejar el evento de clic y detectar el color del píxel seleccionado.
        """
        x = event.x
        y = event.y

        # Obtener el color del píxel seleccionado
        color_pixel_rgb = tuple(self.mapa_coloreado[y, x])

        # Buscar el área correspondiente al color seleccionado
        ruta_pgm = self.encontrar_area_por_color(color_pixel_rgb, self.colores_y_rutas)
        if ruta_pgm:
            print("Área seleccionada: {0} -> {1}".format(color_pixel_rgb, ruta_pgm))

            # Llamar al patrullaje en un hilo separado
            self.ejecutar_patrullaje(ruta_pgm)
        else:
            print("No se encontró un área para el color: {0}".format(color_pixel_rgb))


# Función principal para ejecutar la interfaz
def main():
    # Inicializar el nodo de ROS
    rospy.init_node('seleccionar_pixel_y_mover')

    # Solicitar rutas del mapa y YAML al usuario
    ruta_mapa = "../nav_maps/" + MAP_NAME + ".pgm"
    ruta_yaml = "../nav_maps/" + MAP_NAME + ".yaml"
    ruta_mapa_coloreado = "../output_files/" + MAP_NAME + ".png"
    ruta_csv = "../output_files/" + MAP_NAME + ".csv"
 
    # Crear la ventana de Tkinter
    root = tk.Tk()
    app = MapaApp(root, ruta_mapa, ruta_yaml, ruta_mapa_coloreado, ruta_csv)
    root.mainloop()

if __name__ == '__main__':
    main()
