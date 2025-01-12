import rospy
import csv
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep

def mover_a_goal(x, y):
    """Enviar un goal al robot para moverse a una ubicación específica."""
    # Crear un cliente de acción para el servidor 'move_base'
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Crear un objetivo (goal) para el robot
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # Orientación neutra (sin rotación)

    # Enviar el goal al servidor 'move_base'
    client.send_goal(goal)
    client.wait_for_result()

    # Retornar el resultado del movimiento
    return client.get_result()

def leer_centroides(csv_path):
    """Leer los centroides desde un archivo CSV."""
    centroides = []
    with open(csv_path, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Saltar la cabecera
        for row in reader:
            x = float(row[0])  # Coordenada X en metros
            y = float(row[1])  # Coordenada Y en metros
            centroides.append((x, y))
    return centroides

def mover_a_centroides(centroides):
    """Mover el robot a cada uno de los centroides."""
    for idx, (x, y) in enumerate(centroides):
        rospy.loginfo(f"Moviendo al paso {idx+1} a las coordenadas: ({x}, {y})")
        result = mover_a_goal(x, y)
        
        if result:
            rospy.loginfo(f"Llegamos al paso {idx+1}")
        else:
            rospy.logwarn(f"Falló al llegar al paso {idx+1}")
        
        # Esperar un poco antes de moverse al siguiente objetivo
        rospy.sleep(2)  # Ajusta el tiempo de espera según sea necesario

if __name__ == '__main__':
    # Inicializar el nodo de ROS
    rospy.init_node('mover_robot_centroides', anonymous=True)

    # Ruta al archivo CSV con los centroides
    csv_path = "centroides.csv"  # Cambia la ruta si es necesario

    # Leer los centroides desde el archivo CSV
    centroides = leer_centroides(csv_path)

    # Mover el robot a cada uno de los centroides
    mover_a_centroides(centroides)

