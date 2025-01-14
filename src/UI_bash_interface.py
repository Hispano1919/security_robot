#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from rich.console import Console
from rich.prompt import Prompt
from rich.panel import Panel
from rich.align import Align
from rich.live import Live

import re
import threading
import os
import sys
import termios
import subprocess
import datetime

from APP_config import rooms, TOPIC_COMMAND, TOPIC_LOGS, FOLLOW_ST, SHUTDOWN_ST, MOVE_ST, PATROL_ST, QRFINDER_ST, IDENTIFY_ST
from APP_config import STOP_FOLLOW_CMD, START_DETECTION_CMD, STOP_DETECTION_CMD, START_VOICE_CMD, STOP_VOICE_CMD, STOP_MOVE_CMD

BANNER = """
 [bold white]
 ██████  ██████         ██████  ██████  
 ██   ██      ██       ██            ██ 
 ██████   █████  █████ ██   ███  █████  
 ██   ██ ██            ██    ██ ██      
 ██   ██ ███████        ██████  ███████ 
 [/bold white]                                       
                  _____
                .'/L|__`.
               / =[_]O|` \\
               |"+_____":|
             __:='|____`-:__
            ||[] ||====| []||
            ||[] | |=| | []||
            |:||_|=|U| |_||:|
            |:|||]_=_ =[_||:| 
            | |||] [_][]C|| |
            | ||-'-----`-|| |
            /|\\\_\_|_|_/_//|\\
           |___|   /|\   |___|   
           `---'  |___|  `---'     
                  `---'
"""

class CommandInterface:
    def __init__(self):
        self.console = Console()
        self.running = True
        self.camera_on = True
        self.voice_detection_on = True
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=1)
        rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        self.prompt_thread = None  # Hilo para el Prompt
        self.messages = []  # Historial de mensajes recibidos

    def msg_callback(self, msg):
        """
        Callback que maneja los mensajes recibidos del topic.
        """
        self.messages.append(msg.data)
        if len(self.messages) > 10:
            self.messages.pop(0)
            
    def cmd_callback(self, msg):
        if msg.data == SHUTDOWN_ST:
            self.console.print(":robot: ➡ :wave: [bold red]Apagando robot...[/bold red]")
            self.stop()
            self.running = False

    def handle_command(self, command):
        """
        Procesa los comandos ingresados por el usuario usando palabras clave.
        """
        command = command.lower()  # Convertir a minúsculas para evitar problemas de sensibilidad

        coincidencia = re.match(rooms, command)
        if coincidencia:
            accion = coincidencia.group(1).strip()  # Todo antes del lugar
            lugar = coincidencia.group(2)          # El lugar
        else:
            lugar = None
            accion = command
            
        if "sigue" in command or "sigueme" in command:
            self.console.print(":robot: ➡ :runner: [bold green]Siguiéndote...[/bold green]")
            self.cmd_pub.publish(FOLLOW_ST)
        elif ("busca" in command or "identifica" in command) and ("persona" in command or "personas" in command):
            self.console.print(":robot: ➡ :mag: [bold green]Buscando personas para identificar...[/bold green]")
            self.cmd_pub.publish(IDENTIFY_ST)
        elif "busca" in command and ("qr" in command or "qrs" in command):
            self.console.print(":robot: ➡ :mag_right: [bold green]Buscando QRs...[/bold green]")
            self.cmd_pub.publish(QRFINDER_ST)
        elif "quieto" in command or "quedate" in command:
            self.console.print(":robot: ➡ :stop_sign: [bold yellow]Me quedo quieto.[/bold yellow]")
            self.cmd_pub.publish(STOP_MOVE_CMD)
        elif ("deja" in command and "seguirme" in command) or ("no" in command and "me sigas" in command):
            self.console.print(":robot: ➡ :stop_sign: [bold yellow]No te sigo.[/bold yellow]")
            self.cmd_pub.publish(STOP_FOLLOW_CMD)
        elif lugar and "muevete" in accion or "ve" in accion:
            self.console.print(f":robot: ➡ :house: [bold blue]Yendo a {lugar}...[/bold blue]")
            self.cmd_pub.publish(MOVE_ST + ':' + lugar)
        elif lugar and "patrulla" in accion or "busca" in accion:
            self.console.print(f":robot: ➡ :shield: [bold blue]Patrullando {lugar}...[/bold blue]")
            self.cmd_pub.publish(PATROL_ST + ':' + lugar)
        elif "patrulla" in command:
            self.console.print(f":robot: ➡ :shield: [bold blue]Patrullando la ruta...[/bold blue]")
            self.cmd_pub.publish(PATROL_ST + ':' + "route")
        elif "enciende" in command and "camara" in command:
            self.camera_on = True
            self.console.print(":robot: ➡ :camera: [bold green]Cámara encendida.[/bold green]")
            self.cmd_pub.publish(START_DETECTION_CMD)
        elif "apaga" in command and "camara" in command:
            self.camera_on = False
            self.console.print(":robot: ➡ :camera: [bold red]Cámara apagada.[/bold red]")
            self.cmd_pub.publish(STOP_DETECTION_CMD)
        elif "enciende" in command and "voz" in command:
            self.voice_detection_on = True
            self.cmd_pub.publish(START_VOICE_CMD)
            self.console.print(":robot: ➡ :microphone: [bold green]Detección de voz encendida.[/bold green]")
        elif "apaga" in command and "voz" in command:
            self.voice_detection_on = False
            self.cmd_pub.publish(STOP_VOICE_CMD)
            self.console.print(":robot: ➡ :microphone: [bold red]Detección de voz apagada.[/bold red]")
        elif "exit" in command or "salir" in command:
            self.console.print(":robot: ➡ :wave: [bold red]Saliendo de la interfaz...[/bold red]")
            self.running = False
            self.stop()
            
        elif "adios" in command or "apagar" in command:
            self.cmd_pub.publish(SHUTDOWN_ST)
            self.running = False
            self.stop()
            
        elif "ayuda" in command:
            self.console.print(
                Panel(
                    "[bold green]Comandos Disponibles:[/bold green]\n"
                    ":runner: [bold blue] sigueme[/bold blue] - R2-G2 te seguirá\n"
                    ":stop_sign: [bold blue] quieto[/bold blue] - R2-G2 se quedará en su posición\n"
                    ":shield: [bold blue] patrulla <lugar>[/bold blue] - Patrulla un lugar específico\n"
                    ":camera: [bold blue] enciende/apaga cámara[/bold blue] - Controla la cámara\n"
                    ":microphone: [bold blue] enciende/apaga voz[/bold blue] - Controla detección de voz\n"
                    ":wave: [bold blue] adios[/bold blue] - Apaga R2-G2",
                    title="[bold cyan]Ayuda de Comandos - R2-G2[/bold cyan]",
                    expand=False,
                )
            )
        else:
            self.console.print(f":warning: [bold red]Comando desconocido: {command}[/bold red]")

    def prompt_loop(self):
        """
        Hilo para manejar el prompt interactivo.
        """
        os.system('clear || cls') 
        rospy.init_node("GUICmdNode")
        self.console.print(Align.center(Panel("[bold cyan]Interfaz de Comandos Iniciada[/bold cyan]\nEscribe 'salir' para salir.",
                                 title="[bold blue]Control de R2-G2[/bold blue]", expand=False), vertical="middle"))
        self.log_pub.publish("[INFO] GUI: Started node")
        
        while self.running:
            try:
                command = Prompt.ask("[bold magenta]Ingresa un comando[/bold magenta]")
                self.handle_command(command)
            except Exception as e:
                self.console.print(f":warning: [bold red]Error en el prompt: {e}[/bold red]")

        rospy.signal_shutdown("Finish")
        
    def app_log_viewer(self):
        """
        Función para visualizar mensajes recibidos del topic TOPIC_LOGS.
        Mantiene un historial de los últimos 10 mensajes.
        """
        os.system('clear || cls') 
        rospy.init_node("GUISuscriberNode")
        self.console.print(Align.center(Panel("[bold cyan]Historial de Mensajes[/bold cyan]",
                                 title="[bold yellow]APP Log session[/bold yellow]", expand=False), vertical="middle"))

        def format_message(message):
            """
            Formatea un mensaje recibido para aplicar colores al timestamp, tag, estado y contenido.
            """
            # Regex para dividir el mensaje en partes: timestamp, tag, estado y contenido
            match = re.match(r"(\[\d{2}:\d{2}:\d{2}\])\s(\[.*?\])\s(.*?:)\s(.*)", message)
            if match:
                timestamp, tag, state, description = match.groups()
                if "[ERROR]" in tag:
                    return f"[red]{timestamp}[/red] [red]{tag}[/red] [red]{state}[/red] {description}"
                elif "[WARNING]" in tag:
                    return f"[cyan]{timestamp}[/cyan] [yellow]{tag}[/yellow] [white]{state}[/white] {description}"
                elif "[EVENT]" in tag:
                    return f"[cyan]{timestamp}[/cyan] [orange]{tag}[/orange] [white]{state}[/white] {description}"
                else:
                    return f"[cyan]{timestamp}[/cyan] [green]{tag}[/green] [yellow]{state}[/yellow] {description}"

            else:
                return message
            
        # Función de callback para manejar mensajes
        def callback(msg):
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            self.messages.append(format_message(f"[{timestamp}] {msg.data}"))
            if len(self.messages) > 20:
                self.messages.pop(0)
            
            os.system('clear' if os.name == 'posix' else 'cls')
            self.console.print(Align.center(Panel("[bold cyan]Historial de Mensajes[/bold cyan]",
                                 title="[bold yellow]APP Log session[/bold yellow]", expand=False), vertical="middle"))
            # Imprimir el historial de mensajes
            self.console.print("\n".join(self.messages[-20:]))

        
            
        # Suscribirse al topic
        rospy.Subscriber(TOPIC_LOGS, String, callback)

        # Mostrar los mensajes en tiempo real
        while not rospy.is_shutdown():
            rospy.sleep(0.1)  # Evitar consumo excesivo de recursos

        self.console.print("[bold red]Finalizando visualización de mensajes.[/bold red]")
        rospy.signal_shutdown("Finish")
        
    def show_ascii_banner(self):
        # Mostrar el banner en la consola
        self.console.print(Align.center(Panel(Align.center(Panel(BANNER, expand=False, border_style="red"), vertical="middle"))))
        
        # Esperar a que el usuario presione cualquier tecla
        self.console.print("\n [bold red] ROBOT GUARDIAN [/bold red]", justify="center")
        self.console.print("[bold cyan]\nPresiona cualquier tecla para comenzar.[/bold cyan]", justify="center")
        self.wait_for_keypress()
    
    def wait_for_keypress(self):
        """
        Espera a que el usuario presione cualquier tecla.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            os.read(fd, 1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
    def start(self):
        """
        Inicia el bucle para recibir comandos.
        """
        os.system('clear || cls')   # Clear console
        self.show_ascii_banner()  # Mostrar el banner de inicio
        os.system('clear || cls') # Clear console
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        os.environ["PYTHONPATH"] = f"{script_dir}:{os.environ.get('PYTHONPATH', '')}"

        # Inicia una sesión de tmux
        subprocess.run(["tmux", "new-session", "-d", "-s", "r2g2_session"])

        # Divide la sesión en dos paneles horizontales
        subprocess.run(["tmux", "split-window", "-h", "-t", "r2g2_session"])

        # Ejecuta el script principal en el primer panel
        
        subprocess.run([
            "tmux", "send-keys", "-t", "r2g2_session:0.0",
            "python3 -c 'from UI_bash_interface import CommandInterface; CommandInterface().app_log_viewer()'", "C-m"
        ])
        
        
        
        subprocess.run([
            "tmux", "send-keys", "-t", "r2g2_session:0.1",
            "python3 -c 'from UI_bash_interface import CommandInterface; CommandInterface().prompt_loop()'", "C-m"
        ])
        
        # Adjunta la sesión de tmux para mostrarla al usuario
        subprocess.run(["tmux", "attach-session", "-t", "r2g2_session"])
        # Ejecuta la función de visualización de mensajes en el segundo panel
        
        
        #self.console.print(Align.center(Panel("[bold cyan]Interfaz de Comandos Iniciada[/bold cyan]\nEscribe 'salir' para salir.",
        #                         title="[bold blue]Control de R2-G2[/bold blue]", expand=False), vertical="middle"))
        #self.prompt_thread = threading.Thread(target=self.prompt_loop, daemon=True)
        #self.prompt_thread.start()

        self.stop()
    
    def stop(self):
        subprocess.run(["tmux", "kill-session", "-t", "r2g2_session"])
        subprocess.run(["tmux", "kill-session", "-t", "r2g2_session"])
        print("Sesión de tmux 'r2g2_session' terminada.")
        os.system('clear || cls') 
        rospy.signal_shutdown("Saliendo del programa")
        
        
if __name__ == "__main__":
    rospy.init_node('bash_interface')
    
    interface = CommandInterface()
    interface.start()
    
    rospy.spin