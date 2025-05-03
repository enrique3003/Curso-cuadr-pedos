#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi
from tf.transformations import euler_from_quaternion
import time

waypoints = [(1, -0.25), (3, -0.25), (3.5, 1), (7, 1)]
current_waypoint_index = 0

rospy.init_node('waypoint_navigation', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

current_x = 0.0
current_y = 0.0
current_theta = 0.0

def odom_callback(msg):
    global current_x, current_y, current_theta
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    _, _, current_theta = euler_from_quaternion([
        orientation_q.x, orientation_q.y,
        orientation_q.z, orientation_q.w
    ])

def move_to_waypoint(waypoint, linear_base_speed):
    dx = waypoint[0] - current_x
    dy = waypoint[1] - current_y
    distance = sqrt(dx**2 + dy**2)
    angle_to_goal = atan2(dy, dx)
    angle_diff = angle_to_goal - current_theta

    # Normalizar ángulo
    while angle_diff > pi:
        angle_diff -= 2 * pi
    while angle_diff < -pi:
        angle_diff += 2 * pi

    linear_speed = min(linear_base_speed * distance, linear_base_speed)
    angular_speed = 0.4 * angle_diff
    angular_speed = max(min(angular_speed, 0.35), -0.35)

    cmd_vel = Twist()
    cmd_vel.linear.x = linear_speed
    cmd_vel.angular.z = angular_speed
    cmd_vel_pub.publish(cmd_vel)

def navigate():
    global current_waypoint_index

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10)

    retries = 0
    previous_distance = None
    stuck_threshold = 0.1
    speed_levels = [0.3, 0.5, 0.6, 0.7]
    previous_check_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time()

        waypoint = waypoints[current_waypoint_index]
        dx = waypoint[0] - current_x
        dy = waypoint[1] - current_y
        distance = sqrt(dx**2 + dy**2)

        # Usar velocidad según intento actual
        speed = speed_levels[min(retries, len(speed_levels) - 1)]
        move_to_waypoint(waypoint, speed)

        # Si hemos llegado al waypoint
        if distance < 0.2:
            rospy.loginfo(f"Waypoint {waypoint} alcanzado.")
            current_waypoint_index += 1
            retries = 0
            previous_distance = None
            continue

        if current_time - previous_check_time > 8.0:
            if previous_distance is not None and abs(distance - previous_distance) < stuck_threshold:
                retries += 1
                rospy.logwarn(f"Aparentemente atascado. Reintento {retries} con velocidad {speed} m/s")

                if retries >= len(speed_levels):
                    rospy.logerr("No se pudo superar el obstáculo. Saltando al siguiente waypoint.")
                    current_waypoint_index += 1
                    retries = 0
                    previous_distance = None

                previous_check_time = current_time
                previous_distance = distance  # Solo actualizamos después del chequeo
            else:
                retries = 0
                previous_check_time = current_time
                previous_distance = distance


if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass


