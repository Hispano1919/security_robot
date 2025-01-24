#!/usr/bin/env python3
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
import re
import subprocess  # Para llamar al script
#import patrullaje  # Importar el módulo directamente
from cv_bridge import CvBridge
from sensor_msgs.msg import Image  # Asegurarse de que la clase Image de ROS se importa correctamente
from std_msgs.msg import String

from APP_config import rooms, TOPIC_COMMAND, TOPIC_LOGS, FOLLOW_ST, SHUTDOWN_ST, MOVE_ST, PATROL_ST, MAP_NAME, TOPIC_RGBCAM
from APP_config import STOP_FOLLOW_CMD, START_DETECTION_CMD, STOP_DETECTION_CMD, START_VOICE_CMD, STOP_VOICE_CMD, PATROL_ST, MOVE_ST

class MapaApp:
    def __init__(self, master, ruta_mapa, ruta_yaml, ruta_mapa_coloreado, ruta_csv):
        self.master = master
        self.ruta_mapa = ruta_mapa
        self.ruta_yaml = ruta_yaml
        self.ruta_mapa_coloreado = ruta_mapa_coloreado
        self.ruta_csv = ruta_csv

        # Inicializar CvBridge para convertir mensajes ROS a imágenes OpenCV
        self.bridge = CvBridge()
        ##################################################################################
        # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar #
        ##################################################################################

        # Crear un Frame para contener la cámara
        self.frame_camera = tk.Frame(self.master, bg="lightgrey", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.frame_camera.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Configurar el comportamiento del grid para que el Frame se expanda
        self.master.grid_rowconfigure(0, weight=1)  # Expande la fila 0
        self.master.grid_columnconfigure(1, weight=1)  # Expande la columna 1

        # Crear un contenedor (Frame) interno para centrar el canvas de la cámara
        self.frame_camera_center = tk.Frame(self.frame_camera, bg="black")
        self.frame_camera_center.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Configurar el comportamiento del grid para que el contenedor se expanda
        self.frame_camera.grid_rowconfigure(0, weight=1)  # Expande la fila del contenedor
        self.frame_camera.grid_columnconfigure(0, weight=1)  # Expande la columna del contenedor

        # Crear un Canvas para mostrar la cámara dentro del contenedor centrado
        self.canvas_camera = tk.Canvas(self.frame_camera_center, width=640, height=480)
        self.canvas_camera.grid(row=0, column=0, padx=10, pady=10)

        # Asegúrate de que el Canvas se expanda dentro del Frame
        self.frame_camera_center.grid_rowconfigure(0, weight=1)  # Expande la fila del canvas
        self.frame_camera_center.grid_columnconfigure(0, weight=1)  # Expande la columna del canvas

        # Suscribirse al topic de la cámara
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_camera)

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

        # Crear un Frame para contener el mapa y centrarlo
        self.frame_mapa = tk.Frame(self.master, bg="lightgrey", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.frame_mapa.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Configurar el grid para que el Frame del mapa se expanda
        self.master.grid_rowconfigure(0, weight=1)
        self.master.grid_columnconfigure(0, weight=1)

        # Crear un contenedor (Frame) interno para centrar el Canvas del mapa
        self.frame_mapa_center = tk.Frame(self.frame_mapa, bg="black")
        self.frame_mapa_center.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Configurar el grid para que el contenedor interno se expanda
        self.frame_mapa.grid_rowconfigure(0, weight=1)
        self.frame_mapa.grid_columnconfigure(0, weight=1)

        # Crear el Canvas de Tkinter para mostrar el mapa dentro del contenedor centrado
        self.canvas_mapa = tk.Canvas(self.frame_mapa_center, width=self.mapa_tk.width(), height=self.mapa_tk.height())
        self.canvas_mapa.create_image(0, 0, anchor=tk.NW, image=self.mapa_tk)
        self.canvas_mapa.grid(row=0, column=0, padx=10, pady=10)

        # Asegúrate de que el Canvas se expanda dentro del Frame
        self.frame_mapa_center.grid_rowconfigure(0, weight=1)
        self.frame_mapa_center.grid_columnconfigure(0, weight=1)

        # Repite lo mismo para el Canvas del mapa coloreado
        self.frame_mapa_coloreado = tk.Frame(self.master, bg="lightgrey", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.frame_mapa_coloreado.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.master.grid_rowconfigure(1, weight=1)

        self.frame_mapa_coloreado_center = tk.Frame(self.frame_mapa_coloreado, bg="black")
        self.frame_mapa_coloreado_center.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.frame_mapa_coloreado.grid_rowconfigure(0, weight=1)
        self.frame_mapa_coloreado.grid_columnconfigure(0, weight=1)

        self.canvas_mapa_coloreado = tk.Canvas(self.frame_mapa_coloreado_center, width=self.mapa_coloreado_tk.width(), height=self.mapa_coloreado_tk.height())
        self.canvas_mapa_coloreado.create_image(0, 0, anchor=tk.NW, image=self.mapa_coloreado_tk)
        self.canvas_mapa_coloreado.grid(row=0, column=0, padx=10, pady=10)

        self.frame_mapa_coloreado_center.grid_rowconfigure(0, weight=1)
        self.frame_mapa_coloreado_center.grid_columnconfigure(0, weight=1)

        #####################3

        # Crear un Frame para contener los botones
        self.frame_botones = tk.Frame(self.master, bg="lightgrey", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.frame_botones.grid(row=1, column=1, sticky="nsew", padx=10, pady=10, columnspan=1)

        # Configurar el grid del Frame para centrar los botones
        self.frame_botones.grid_rowconfigure(0, weight=1)
        self.frame_botones.grid_rowconfigure(1, weight=1)
        self.frame_botones.grid_rowconfigure(2, weight=1)
        self.frame_botones.grid_columnconfigure(0, weight=1)

        # Añadir botones al Frame (haciendo que sean cuadrados)
        button_size = 10  # Tamaño de los botones (ancho y alto)

        self.boton1 = tk.Button(self.frame_botones, text="Stop\nRobot", command=self.funcion_boton1, bg="#007BFF", fg="white", width=button_size, height=button_size)
        self.boton2 = tk.Button(self.frame_botones, text="Go\nHome", command=self.funcion_boton2, bg="#28A745", fg="white", width=button_size, height=button_size)
        self.boton3 = tk.Button(self.frame_botones, text="Centroides", command=self.funcion_boton3, bg="#DC3545", fg="white", width=button_size, height=button_size)

        # Colocar los botones en un grid y centrarlos
        self.boton1.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.boton2.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.boton3.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")


        #####################
        
        # Integrar la consola de comandos
        self.command_frame = tk.Frame(self.master, bg="lightgrey", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.command_frame.grid(row=1, column=2, columnspan=2, padx=10, pady=10, sticky="nsew")

        # Configurar el grid para que la consola ocupe todo el espacio disponible
        self.command_frame.grid_rowconfigure(0, weight=1)  # La consola de texto
        self.command_frame.grid_rowconfigure(1, weight=0)  # La entrada de texto
        self.command_frame.grid_rowconfigure(2, weight=0)  # El botón
        self.command_frame.grid_columnconfigure(0, weight=1)

        # Crear widgets para la consola de comandos
        self.console = tk.Text(self.command_frame, height=10, width=60)
        self.console.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)  # Expande para ocupar espacio

        self.command_entry = tk.Entry(self.command_frame, width=60)
        self.command_entry.grid(row=1, column=0, sticky="ew", padx=5, pady=5)  # Expande horizontalmente

        self.send_button = tk.Button(self.command_frame, text="Enviar", command=self.send_command)
        self.send_button.grid(row=2, column=0, sticky="ew", padx=5, pady=5)  # Botón centrado horizontalmente

        ###############################################33

        # Crear un Frame para la imagen
        self.frame_imagen = tk.Frame(self.master, bg="white", padx=10, pady=10, borderwidth=2, relief="ridge")
        self.frame_imagen.grid(row=0, column=2, padx=10, pady=10, sticky="nsew")  # Ubicar el Frame en la celda 0,2

        # Configurar el grid para que la celda del Frame se expanda
        self.master.grid_rowconfigure(0, weight=1)
        self.master.grid_columnconfigure(2, weight=1)

        # Configurar el Frame para expandirse dentro de su celda
        self.frame_imagen.grid_rowconfigure(0, weight=1)
        self.frame_imagen.grid_columnconfigure(0, weight=1)

        # Crear un Canvas dentro del Frame
        self.canvas_imagen = tk.Canvas(self.frame_imagen, bg="lightgrey")
        self.canvas_imagen.grid(row=0, column=0, sticky="nsew")  # Hacer que el Canvas ocupe todo el Frame

        # Cargar la imagen
        self.imagen_original = PILImage.open("../images/robot.png")  # Reemplaza con la ruta de tu imagen

        # Redibujar la imagen para que ocupe todo el Frame dinámicamente
        def actualizar_imagen(event):
            # Obtener el tamaño actual del Frame
            ancho = event.width
            alto = event.height

            # Redimensionar la imagen original al tamaño del Frame
            imagen_redimensionada = self.imagen_original.resize((ancho, alto), PILImage.ANTIALIAS)
            self.imagen_tk = ImageTk.PhotoImage(imagen_redimensionada)

            # Limpiar el Canvas y mostrar la imagen redimensionada
            self.canvas_imagen.delete("all")
            self.canvas_imagen.create_image(0, 0, anchor=tk.NW, image=self.imagen_tk)

        # Vincular el evento de redimensionamiento al Canvas
        self.canvas_imagen.bind("<Configure>", actualizar_imagen)

        ##################################################################################
        # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar # Copiar #
        ##################################################################################
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
        self.cmd_pub.publish(PATROL_ST + ':' + "route")
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
        try:
            # Rutas fijas (ajusta según sea necesario)
            #ruta_yaml = "casa_3.yaml"
            #espaciado = 10  # Configuración fija o ajustable
            #patrullaje.main(ruta_pgm, ruta_yaml, espaciado)  # Llamada directa a la función
            # Obtener el nombre del área desde el archivo PGM

            nombre_area = ruta_pgm.split('/')[-1].replace('.pgm', '')
            resultado = re.search(r'area_\d+', nombre_area).group()
            print(resultado)
            # Publicar el comando de patrullaje
            self.cmd_pub.publish(PATROL_ST + ':' + resultado)

        except Exception as e:
            print("Error al ejecutar el patrullaje: {0}".format(e))

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
                #self.ejecutar_movimiento(coord_x_inicio, coord_y_inicio, theta)
                
                # Enviar el comando al robot
                command = MOVE_ST + ":{:.2f},{:.2f},{:.2f}".format(coord_x_inicio, coord_y_inicio, theta)
                self.cmd_pub.publish(command)
               
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
