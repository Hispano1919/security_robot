<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- GETTING STARTED -->
## Introducción

Este repositorio contiene el paquete de ROS security_robot para la asignatura Robots Móviles.

### Prerequisitos

Estos son los siguientes requisitos que se necesitan para poder usar el paquete:
1. ROS Noetic para Ubuntu: [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Turtlebot2: 
Instalar el simulador de Turtlebot 2

Estos robots sí los tenemos en el laboratorio, de modo que lo que pruebes en el simulador luego podrás probarlo con los robots reales. Por desgracia, no tienen soporte oficial para Noetic, lo que nos va a obligar a compilar los fuentes.

Para descargar los fuentes de los diversos repositorios necesarios de manera automática necesitas bajarte primero este fichero tb2.rosinstall. Bájatelo y déjalo en tu directorio $HOME (tu directorio personal, o sea /home/tu_nombre_de_usuario).

* Instrucciones
  ```sh
  #Dependencias de paquetes de Ubuntu
  sudo apt install libusb-dev libftdi-dev python-is-python3 pyqt5-dev-tools

  #Dependencias de paquetes de Noetic
  sudo apt install ros-noetic-openslam-gmapping ros-noetic-joy ros-noetic-base-local-planner ros-noetic-move-base

  #Directorio para los fuentes
  mkdir $HOME/tb2_ws
  cd $HOME/tb2_ws

  #Se baja los fuentes de los repos especificados en el .rosinstall
  wstool init src ../tb2.rosinstall

  #Arregla un problema de dependencias
  rm -rf src/ar_track_alvar/ar_track_alvar

  #Compilar.
  catkin_make_isolated
  ```

3. Workspace de catkin previamente creado: [http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
4. Gazebo: [https://classic.gazebosim.org/tutorials?tut=install_ubuntu](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) 
5. Mediapipe: [https://pypi.org/project/mediapipe/](https://pypi.org/project/mediapipe/)
6. Pynput: 
   ```sh
   pip install pynput
   ```
### Instalación

Para instalar el paquete security_robot:

1. Clonar el repositorio dentro de la carpeta src del WS de catkin:
   ```sh
   git clone https://github.com/Hispano1919/security_robot.git
   ```
2. Cambiar el nombre del directorio descargado a security_robot (sólo si tiene otro nombre, como por ejemplo: security_robot-master)

3. Desde el directorio raiz del WS de catkin:
   ```sh
   # Compila el paquete de ROS
   catkin build security_robot

   # Actualiza el entorno
   source devel/setup.bash
   ```

Con estos pasos el paquete ya debería estar correctamente compilado y listo para su uso.
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Uso (WIP)

Para usar este paquete es conveniente seguir estos pasos, cada uno en una terminal diferente (es posible que se necesite hacer, desde el directorio raiz del Workspace de catkin, source devel/setup.bash, al abrir cada terminal):

1. Lanzar simulación en gazebo:
   ```sh
   roslaunch security_robot security_world.launch 
   ```
2. Lanzar nodos de control visual:
   ```sh
   roslaunch security_robot visual_control.launch
   ```
3. Enviar mensaje de "Start" al robot:
   ```sh
   rostopic pub /robot_command std_msgs/String "start_follow"
   ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## Participantes

- José Llorens Megías
- Asahel Hernández Torné

<p align="right">(<a href="#readme-top">back to top</a>)</p>

