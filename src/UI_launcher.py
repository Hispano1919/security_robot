#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import customtkinter as ctk
from tkinter import messagebox
from PIL import Image, ImageTk
import rospkg

# Nombre del paquete ROS
PACK_NAME = "security_robot"

def install_requirements():
    requirements_path = os.path.join(package_path, "requirements.txt")
    try:
        subprocess.run(["pip3", "install", "-r", requirements_path], check=True)
        messagebox.showinfo("Instalación", "Requisitos instalados correctamente.")
    except subprocess.CalledProcessError:
        messagebox.showerror("Error", "Error instalando los requisitos.")

def launch_simulation():
    sim_option = "-s" if not simulation.get() else ""
    move_option = "move_person" if move_person.get() else ""
    rviz_option = "rviz" if rviz.get() else ""

    launch_script_path = os.path.join(package_path, "src", "run.sh")

    command = [
        "gnome-terminal", "--",
        "bash", "-c",
        f"{launch_script_path} {sim_option} --world {world_name.get()} {mode.get().lower()} {move_option} {rviz_option}; exec bash"
    ]

    try:
        subprocess.run(command, check=True)
        messagebox.showinfo("Ejecución", "Simulación lanzada correctamente.")
    except subprocess.CalledProcessError:
        messagebox.showerror("Error", "No se pudo lanzar la simulación.")

def get_world_names():
    world_dir = os.path.join(package_path, "worlds")
    if not os.path.exists(world_dir):
        os.makedirs(world_dir)  # Crear la carpeta si no existe
    world_files = [f for f in os.listdir(world_dir) if f.endswith(".world")]
    return [os.path.splitext(f)[0] for f in world_files]


# Obtener la ruta del paquete usando rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path(PACK_NAME)

# Crear la ventana principal
root = ctk.CTk()
root.title("R2-G2")
window_width = 700
window_height = 700
root.geometry(f"{window_width}x{window_height}")
root.configure(fg_color="#d9d9d9")  # Color de fondo gris

# Cargar imagen de fondo
bg_image_path = os.path.join(package_path, "images", "gui.png")

try:
    bg_image = ctk.CTkImage(
        dark_image=Image.open(bg_image_path),  # Imagen para modo oscuro
        light_image=Image.open(bg_image_path),  # Imagen para modo claro
        size=(window_width, window_height)  # Tamaño ajustado
    )

    bg_label = ctk.CTkLabel(root, image=bg_image, text="")
    bg_label.place(x=0, y=0, relwidth=1, relheight=1)
    
except FileNotFoundError:
    messagebox.showerror("Error", f"No se encontró la imagen {bg_image_path}")
    root.destroy()


# Cargar imagen para el botón de salida
exit_image = ctk.CTkImage(
    dark_image=Image.open(package_path + "/images/exit.png"),  # Imagen para modo oscuro
    light_image=Image.open(package_path + "/images/exit.png"),  # Imagen para modo oscuro
    size=(181, 59)
)

start_image = ctk.CTkImage(
    dark_image=Image.open(package_path + "/images/start.png"),  # Imagen para modo oscuro
    light_image=Image.open(package_path + "/images/start.png"),  # Imagen para modo oscuro
    size=(181, 59)
)
# Obtener nombres de mundos desde la carpeta dentro del paquete ROS
world_options = get_world_names()
world_name = ctk.StringVar(value=world_options[0] if world_options else "")

# Modos de ejecución
modes = ["Minimal", "Light", "Heavy", "Explore", "Segmentation"]
mode = ctk.StringVar(value=modes[0])

move_person = ctk.BooleanVar(value=False)
rviz = ctk.BooleanVar(value=False)
simulation = ctk.BooleanVar(value=True)


# Menú desplegable para selección de mundos
if world_options:
    world_selector = ctk.CTkComboBox(
        root,
        variable=world_name,
        values=world_options,
        hover=True,
        width=173,  # Ancho del ComboBox
        height=37,  # Altura del ComboBox
        fg_color="#d9d9d9",  # Color de fondo
        border_color="#c00000",  # Color del borde
        border_width=1,  # Grosor del borde
        dropdown_fg_color="#8d8d8d",  # Color de fondo del menú desplegable
        dropdown_text_color="black",  # Color del texto en las opciones desplegables
        text_color="black",  # Color del texto seleccionado
        button_color="#8d8d8d",  # Color del botón desplegable
        button_hover_color="#white",  # Color del botón al pasar el mouse
        corner_radius=10,  # Bordes redondeados
        font=("Arial", 16),  # Fuente del texto seleccionado
        dropdown_font=("Arial", 14),  # Fuente del menú desplegable
        state="readonly",  # Deshabilita la edición manual
        justify="center",
        dropdown_hover_color="#c00000",
    )
    world_selector.place(x=72, y=211)

# Menú desplegable para selección de modo
mode_selector = ctk.CTkComboBox(
        root,
        variable=mode,
        values=modes,
        width=173,  # Ancho del ComboBox
        height=37,  # Altura del ComboBox
        fg_color="#d9d9d9",  # Color de fondo
        border_color="#c00000",  # Color del borde
        border_width=1,  # Grosor del borde
        dropdown_fg_color="#8d8d8d",  # Color de fondo del menú desplegable
        dropdown_text_color="black",  # Color del texto en las opciones desplegables
        text_color="black",  # Color del texto seleccionado
        button_color="#8d8d8d",  # Color del botón desplegable
        button_hover_color="#c00000",  # Color del botón al pasar el mouse
        corner_radius=10,  # Bordes redondeados
        font=("Arial", 16),  # Fuente del texto seleccionado
        dropdown_font=("Arial", 14),  # Fuente del menú desplegable
        state="readonly",  # Deshabilita la edición manual
        justify="center",
        dropdown_hover_color="#c00000",
        hover=True
    )
mode_selector.place(x=453, y=211)

# Crear Checkbutton (simulado con CTkSwitch para mantener la transparencia)
move_person_switch = ctk.CTkSwitch(
    root,
    text="",  # Texto opcional
    variable=move_person,
    onvalue=True,
    offvalue=False,
    width=35,  # Ancho total del switch
    height=33,  # Altura total del switch
    corner_radius=20,  # Bordes más redondeados
    border_width=2,  # Grosor del borde
    fg_color="#8d8d8d",  # Color cuando está activado
    progress_color="lightgreen",  # Color de la barra de progreso
    button_color="white",  # Color del botón
    button_hover_color="gray"  # Color del botón al pasar el cursor
)
move_person_switch.place(x=358, y=306)

rviz_switch = ctk.CTkSwitch(
    root,
    text="",  # Texto opcional
    variable=rviz,
    onvalue=True,
    offvalue=False,
    width=35,  # Ancho total del switch
    height=33,  # Altura total del switch
    corner_radius=20,  # Bordes más redondeados
    border_width=2,  # Grosor del borde
    fg_color="#8d8d8d",  # Color cuando está activado
    progress_color="lightgreen",  # Color de la barra de progreso
    button_color="white",  # Color del botón
    button_hover_color="gray"  # Color del botón al pasar el cursor
)
rviz_switch.place(x=305, y=387)

simulation_switch = ctk.CTkSwitch(
    root,
    text="",  # Texto opcional
    variable=simulation,
    onvalue=True,
    offvalue=False,
    width=35,  # Ancho total del switch
    height=33,  # Altura total del switch
    corner_radius=20,  # Bordes más redondeados
    border_width=2,  # Grosor del borde
    fg_color="#8d8d8d",  # Color cuando está activado
    progress_color="lightgreen",  # Color de la barra de progreso
    button_color="white",  # Color del botón
    button_hover_color="gray"  # Color del botón al pasar el cursor
)
simulation_switch.place(x=358, y=468)

start_button = ctk.CTkButton(
    root, 
    text="",  # Sin texto para hacerlo más visualmente limpio
    image=start_image,  # Imagen del botón
    command=launch_simulation,
    fg_color="#196b24",  # Fondo transparente
    
    hover_color="#0e3a14",
    width=181, 
    height=59,
    border_width=0,  # Sin borde
    corner_radius=10,  # Bordes redondeados
    border_spacing=0
)
start_button.place(x=256, y=562)

exit_button = ctk.CTkButton(
    root, 
    text="",  # Sin texto para hacerlo más visualmente limpio
    image=exit_image,  # Imagen del botón
    command=root.quit,
    fg_color="#c00000",  # Fondo transparente
    
    hover_color="#660000",
    width=181, 
    height=59,
    border_width=0,  # Sin borde
    corner_radius=10,  # Bordes redondeados
    border_spacing=0
)
exit_button.place(x=256, y=628)
# Ejecutar la aplicación
root.mainloop()
