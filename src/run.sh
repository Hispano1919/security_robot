#!/bin/bash

# Variables por defecto
MODE="light"
MOVEPERSON="false"
RVIZ="false"
WORLD_NAME="casa3"
SIMULATION="true"


# Fuente de ROS (ajustar según tu configuración)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/ROS_WS/devel/setup.bash

show_help() {
  echo "Invalid argument: $1"
  echo "Use: $0 [--world <world_file>] <option> <move_person> <rviz>"
  echo "Options:"
  echo "  heavy - Launches everything"
  echo "  light - Launches navigation, follow, voice control and GUI"
  echo "  minimal - Launches navigation, follow and console"
  echo "  explore - Launches exploration "
  echo "  segmentation - Launches map segmentation "
  echo "Flags:"
  echo "  move_person - Launches move_person node"
  echo "  rviz - Launches RVIZ with navigation"
  exit 1
}

# Procesar los argumentos
while [[ $# -gt 0 ]]; do
  case "$1" in
    -s)
      SIMULATION="false"
      shift
      ;;
    -w)
      WORLD_NAME="$2"
      shift 2
      ;;
    --world)
      WORLD_NAME="$2"
      shift 2
      ;;
    heavy|light|minimal|explore|segmentation)
      MODE="$1"
      shift
      ;;
    move_person)
      MOVEPERSON="true"
      shift
      ;;
    rviz)
      RVIZ="true"
      shift
      ;;
    help)
      show_help
      shift
      ;;
    *)
      show_help "$1"
      shift
      ;;
  esac
done

if [ "$MODE" == "segmentation" ]; then
  SIMULATION="false"
  echo "Skipping world and navigation launch for map segmentation"
fi

if [ "$SIMULATION" == "true" ]; then
  echo "Launching World..."
  gnome-terminal --title="WORLD" -- bash -c "roslaunch security_robot world.launch world_name:=$WORLD_NAME" &
  WORLD_PID=$!  # Guardar el PID del terminal
  # Añadir un retardo para permitir que el mundo se cargue
  echo "Waiting for World to load..."
  sleep 10
  # Lanzar navegación solo si no está en modo "explore"
  if [ "$MODE" != "explore" ]; then
    echo "Launching Navigation..."
    if [ "$RVIZ" == "true" ]; then
      gnome-terminal --title="NAV" -- bash -c "roslaunch security_robot navigation.launch world_name:=$WORLD_NAME rviz:=true" &
    else
      gnome-terminal --title="NAV" -- bash -c "roslaunch security_robot navigation.launch world_name:=$WORLD_NAME" &
    fi
    sleep 5
  else
    echo "Skipping Navigation launch for explore mode."
  fi
fi


echo "Updating MAP_NAME using APP_main.py..."
python3 APP_map_name_updater.py APP_config.py "$WORLD_NAME"

sleep 1
# Seleccionar el archivo .launch según el argumento
case "$MODE" in
  heavy)
    echo "Launching Nodes: Heavy..."
    gnome-terminal --title="NODES_HEAVY" -- bash -c "roslaunch security_robot nodes_heavy.launch world_name:=$WORLD_NAME" &
    ;;
  light)
    echo "Launching Nodes: Light..."
    gnome-terminal --title="NODES_LIGHT" -- bash -c "roslaunch security_robot nodes_light.launch world_name:=$WORLD_NAME" &
    ;;
  minimal)
    echo "Launching Nodes: Minimal..."
    gnome-terminal --title="NODES_MINIMAL" -- bash -c "roslaunch security_robot nodes_minimal.launch world_name:=$WORLD_NAME" &
    ;;
  explore)
    echo "Launching exploration pipeline"
    gnome-terminal --title="EXPLORATION" -- bash -c "roslaunch security_robot exploration.launch" &
    ;;
  segmentation)
    echo "Launching map segmentation pipeline"
    gnome-terminal --title="MAP_SEGMENTATION" -- bash -c "rosrun security_robot APP_map_processor.py" &
    ;;
esac
sleep 1
if [ "$MOVEPERSON" == "true" ]; then
  echo "Executing move_person node..."
  gnome-terminal --title="MOVE_PERSON" -- bash -c "rosrun security_robot APP_move_person.py" &
fi
sleep 1
# Esperar a que el usuario presione una tecla
echo "Press any key to stop all nodes and close terminals..."
read -n 1 -s  # Espera silenciosa hasta que se presione una tecla

# Cerrar todos los terminales lanzados
echo "Closing terminals..."
rostopic pub /robot_cmd std_msgs/String "data: 'shutdown_state'" -1

# Opcional: Cerrar cualquier nodo ROS residual
rosnode kill /gazebo
rosnode kill /gazebo_gui
rosnode cleanup
