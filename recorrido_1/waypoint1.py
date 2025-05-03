#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion  # Importa la función necesaria

# Inicialización de variables
waypoints = [(1, 0), (2.5, 0), (3.5, -1), (7, -1)]  # Waypoints a los que el robot se debe mover
current_waypoint_index = 0  # El índice del waypoint actual

# Inicializamos el nodo de ROS
rospy.init_node('waypoint_navigation', anonymous=True)

# Publicador de las velocidades
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Variable para la posición actual del robot
current_x = 0.0
current_y = 0.0
current_theta = 0.0

# Callback de Odometry para obtener la posición actual
def odom_callback(msg):
    global current_x, current_y, current_theta
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    # Convertimos la orientación de cuaternión a ángulo
    _, _, current_theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

# Función para mover el robot hacia un waypoint
def move_to_waypoint(waypoint):
    global current_x, current_y, current_theta

    # Diferencia de posición (error)
    dx = waypoint[0] - current_x
    dy = waypoint[1] - current_y
    distance = sqrt(dx**2 + dy**2)

    # Calcular el ángulo hacia el waypoint
    angle_to_goal = atan2(dy, dx)
    angle_diff = angle_to_goal - current_theta

    # Ajustar el ángulo de diferencia para estar dentro del rango [-pi, pi]
    while angle_diff > 3.14159:
        angle_diff -= 2 * 3.14159
    while angle_diff < -3.14159:
        angle_diff += 2 * 3.14159

    # Configuración del control proporcional para la velocidad
    linear_speed = 0.6 * distance  # Velocidad proporcional a la distancia
    angular_speed = 0.3 * angle_diff  # Velocidad angular proporcional al error de orientación

    # Limitar velocidades para evitar movimientos demasiado rápidos
    if linear_speed > 0.25:
        linear_speed = 0.25
    if angular_speed > 0.3:
        angular_speed = 0.3

    # Publicar las velocidades
    cmd_vel = Twist()
    cmd_vel.linear.x = linear_speed
    cmd_vel.angular.z = angular_speed
    cmd_vel_pub.publish(cmd_vel)

# Función principal de navegación
def navigate():
    global current_waypoint_index

    # Suscribirse al topic de odometría
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)  # Frecuencia de 10 Hz

    while not rospy.is_shutdown():
        # Obtener el waypoint actual
        current_waypoint = waypoints[current_waypoint_index]

        # Mover al waypoint actual
        move_to_waypoint(current_waypoint)

        # Si estamos suficientemente cerca del waypoint, cambiamos al siguiente
        dx = current_waypoint[0] - current_x
        dy = current_waypoint[1] - current_y
        distance = sqrt(dx**2 + dy**2)

        if distance < 0.2:  # Umbral de cercanía para considerar que hemos llegado al waypoint
            rospy.loginfo(f"Waypoint {current_waypoint} alcanzado")
            current_waypoint_index += 1

            # Si ya hemos visitado todos los waypoints, detenemos el robot
            if current_waypoint_index >= len(waypoints):
                rospy.loginfo("Todos los waypoints alcanzados. Deteniendo robot.")
                break

        rate.sleep()

if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass

