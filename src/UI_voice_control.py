#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import numpy as np
import queue
import json
import pyttsx3
import time
from scipy.signal import resample
import re
import random

from APP_main import TOPIC_COMMAND, TOPIC_LOGS, rooms
from APP_main import FOLLOW_ST, STOP_FOLLOW_CMD, SHUTDOWN_ST, MOVE_ST, PATROL_ST
from APP_main import STOP_DETECTION_CMD, START_DETECTION_CMD, STOP_VOICE_CMD, START_VOICE_CMD, IDENTIFY_CMD

activation_cmds = {"hola", "escucha", "escuchame", "oye"}

class VoiceControl:
    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 120)  # Velocidad del habla
        self.engine.setProperty('volume', 1.0)  # Volumen

        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.log_msg = None
        self.command_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=1)

        self.is_active = True
        self.cafe_on = True
        self.count_cafe = 0
        self.run = True
        self.model_name = "vosk-model-small-es-0.42"
        self.wait_id = True
        self.user_auth = False
        self.timeout = 10

        self.auth_users = {"asahel": "61234", "pepe": "12345", "maria": "54321"}
        self.model_path = rospy.get_param('~voice_model', "../trained_models")

        if self.model_path == "":
            self.model_path = self.model_name
        else:
            self.model_path = self.model_path + '/' + self.model_name

        self.model = Model(self.model_path)
        self.q = queue.Queue()

        # Configuración del micrófono
        self.samplerate = 44100  # Frecuencia del micrófono (44100 Hz)
        sd.default.samplerate = self.samplerate
        sd.default.channels = 1  # Se selecciona un canal mono
        self.recognizer = KaldiRecognizer(self.model, 16000)  

        rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
    
    def log_and_speak(self, message):
        """
        Funcion que habla y escribe por terminal el mensaje pasado por argumento
        """
        rospy.loginfo(message)
        voice_prefix = "[VOICE]: "
        self.log_pub.publish(voice_prefix + message)
        self.engine.say(message)
        self.engine.runAndWait()

    # Callback para manejar el audio en tiempo real
    def audio_callback(self, indata, frames, time, status):
        """
        Audio callback
        """
        # resampleo del audio desde 44khz a 16khz
        downsampled_data = resample(indata, int(len(indata) * 16000 / 44100))
        # Convertir a int16 (escala los valores de flotante a rango de int16)
        int16_data = (downsampled_data * 32767).astype(np.int16)
        self.q.put(int16_data)

    def cmd_callback(self, msg):
        if msg.data == START_VOICE_CMD and not self.is_active:
            self.is_active = True
            rospy.loginfo("VOICE CONTROL: ON")
            self.log_pub.publish("[INFO] VOICE CONTROL: ON")
            
        elif msg.data == STOP_VOICE_CMD and self.is_active:
            self.is_active = False
            rospy.loginfo("VOICE CONTROL: OFF")
            self.log_pub.publish("[INFO] VOICE CONTROL: OFF")

        elif msg.data == SHUTDOWN_ST:
            self.engine.say("Apagando")
            #self.engine.runAndWait()
            rospy.signal_shutdown("Apagando")

        elif msg.data == IDENTIFY_CMD:
            self.id_person()

    def listen_command(self, timeout=30):
        """
        Funcion para aceptar comandos por voz
        """
        start_time = time.time()
        while True:
            try:
                data = self.q.get(timeout=1)  # Evita bloqueos indefinidos
            except queue.Empty:
                continue

            if self.recognizer.AcceptWaveform(data.tobytes()):
                result = json.loads(self.recognizer.Result())
                command = result.get("text", "").lower()
                return command

            if time.time() - start_time > timeout:
                self.log_and_speak("Tiempo de espera agotado.")
                return ""
            
    def id_person(self):
        
        while self.wait_id:
            self.log_and_speak("Por favor, indique su nombre y número de identificación.")
            name = None
            id_number = None

            start_time = time.time()
            while (time.time() - start_time < self.timeout):
                cmd = self.listen_command()
                if cmd:
                    name = cmd.lower()
                    if name in self.auth_users:
                        self.log_and_speak("Ahora, por favor, indique su número de identificación.")

                        start_time = time.time()
                        while (time.time() - start_time < self.timeout):
                            cmd = self.listen_command()
                            
                            if cmd.isdigit():
                                id_number = cmd
                                if name in self.auth_users and self.auth_users[name] == id_number:
                                    self.log_and_speak(f"Identificación correcta. Bienvenido, {name}.")
                                    self.log_pub.publish(f"[USER_ID]: {name} authentication succeded")
                                    self.wait_id = False
                                    self.user_auth = True
                                    self.command_pub.publish(STOP_FOLLOW_CMD)
                                    break
                                
                                else:
                                    self.log_and_speak("Identificación incorrecta. Acceso denegado.")
                                    self.log_pub.publish(f"[USER_ID]: authentication failed: {name}.")
                            else:
                                self.log_and_speak("Por favor, indique un número válido.")
                            if not self.wait_id:
                                break  
                        break

                if not self.wait_id:
                    break
            
        
    def main_loop(self):
        try:
            
            with sd.InputStream(callback=self.audio_callback):
                self.log_and_speak("Escuchando")

                while not rospy.is_shutdown():
                    command = self.listen_command()

                    if "R2-G2" in command or ("r2-g2" in command and activation_cmds in accion):
                        self.is_active = True
                        coincidencia = re.match(rooms, command)
                    
                        if coincidencia:
                            accion = coincidencia.group(1).strip()  # Todo antes del lugar
                            lugar = coincidencia.group(2)           # El lugar
                        else:
                            accion = command
                            lugar = None 

                        start_time = time.time()
                        while (time.time() - start_time < self.timeout) and self.is_active:
                        
                            if "sígueme" or "ven" or "conmigo" in accion:
                                self.log_and_speak("Te sigo.")
                                self.command_pub.publish(FOLLOW_ST)

                            elif "quédate aquí" or "quieto" or "para" in accion:
                                self.log_and_speak("Me quedo quieto.")             
                                self.command_pub.publish(STOP_FOLLOW_CMD)

                            elif lugar != None and not "patrulla" in accion:
                                self.log_and_speak("Me dirijo a " + lugar)
                                self.command_pub.publish(MOVE_ST + ":" + lugar)

                            elif lugar != None and "patrulla" in accion:
                                self.log_and_speak("Patrullando " + lugar)
                                self.command_pub.publish(PATROL_ST + ":" + lugar)

                            elif "vuelve a la estación" or "descansa" in accion:
                                self.log_and_speak("Me dirijo a la estación de carga.")
                                self.command_pub.publish(MOVE_ST + ":" + "estacion")

                            elif "no mires" in accion or ("apaga" in accion and "camara" in accion):
                                self.log_and_speak("De acuerdo, no miro.")
                                self.command_pub.publish(STOP_DETECTION_CMD)

                            elif "mírame" in accion or ("enciende" in accion and "camara" in accion):
                                self.log_and_speak("De acuerdo, enciendo camara.")
                                self.command_pub.publish(START_DETECTION_CMD)

                            elif "patrulla" in accion or ("enciende" in accion and "camara" in accion):
                                self.log_and_speak("De acuerdo, enciendo camara.")
                                self.command_pub.publish(START_DETECTION_CMD)

                            elif "no" in accion and ("oigas" in accion or "escuches" in accion):
                                self.log_and_speak("De acuerdo, no te oigo.")
                                self.is_active = False
                                break

                            elif "adiós" in accion:
                                self.log_and_speak("Adiós")
                                self.command_pub.publish(SHUTDOWN_ST)
                                rospy.signal_shutdown("Apagando")
                                break

                        if self.is_active == False:
                            break

        except Exception as e:
            self.log_and_speak(f"Error durante la ejecución: {e}")

rospy.init_node('Voice_command_node')
processor = VoiceControl()
processor.main_loop()

rospy.spin()    