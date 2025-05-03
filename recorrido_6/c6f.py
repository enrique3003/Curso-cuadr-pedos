#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
import cv2

class WaypointNavigator3D:
    def __init__(self):
        rospy.init_node('pi_navigation_rgb_depth_odom', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/realsense/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/realsense/depth/image_rect_raw', Image, self.depth_callback)

        self.bridge = CvBridge()

        self.waypoints = [(1, 0, 0.0), (2.5, 0, 0.4), (3.0, 0, 0.8), (3.5, 0, 1.2), (4.5, 0, 1.0), (5.5, 0, 0.6), (7, 0, 0.0)]
        self.current_waypoint_index = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.linear_speed_limit = 0.3
        self.angular_speed_limit = 0.25

        self.Kp_lin = 0.35
        self.Kp_ang = 0.7

        self.waypoint_tolerance = 0.12
        self.angle_tolerance = 0.2

        self.depth_image = None
        self.rgb_image = None

        self.STUCK_THRESHOLD = 0.02
        self.STUCK_TIME_WINDOW = 2.5
        self.position_history = []

        self.recovery_attempts = 0
        self.MAX_RECOVERY_ATTEMPTS = 3

        rospy.loginfo("🧭 Navegador PI con RGB+Depth+Odom listo.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.update_position_history()

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"Error al convertir imagen RGB: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logwarn(f"Error al convertir imagen de profundidad: {e}")

    def angle_diff(self, goal_angle):
        diff = goal_angle - self.current_theta
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def update_position_history(self):
        now = time.time()
        self.position_history.append((now, self.current_x, self.current_y))
        self.position_history = [p for p in self.position_history if now - p[0] <= self.STUCK_TIME_WINDOW]

    def is_stuck(self, linear_speed):
        if linear_speed < 0.05:
            return False  # no consideramos atascado si no hay intento real de avance

        if len(self.position_history) < 2:
            return False
        start = self.position_history[0]
        end = self.position_history[-1]
        dist = math.sqrt((end[1] - start[1])**2 + (end[2] - start[2])**2)
        return dist < self.STUCK_THRESHOLD

    def perform_recovery(self):
        rospy.logwarn("🔁 Atasco detectado. Iniciando recuperación...")

        # Paso 1: Retroceder
        cmd = Twist()
        cmd.linear.x = -0.15
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(1.2)

        # Paso 2: Giro lateral
        cmd = Twist()
        cmd.angular.z = 0.4 if self.recovery_attempts % 2 == 0 else -0.4
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(1.0)

        # Paso 3: Impulso
        rospy.loginfo("💥 Impulso con estabilización...")
        cmd = Twist()
        cmd.linear.x = 0.35
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(1.2)

        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(1.5)

        self.recovery_attempts += 1
        self.position_history.clear()

    def navigate(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("✅ Misión completada. Todos los waypoints alcanzados.")
                self.cmd_vel_pub.publish(Twist())
                break

            target_x, target_y, _ = self.waypoints[self.current_waypoint_index]
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)

            angle_to_goal = math.atan2(dy, dx)
            diff_angle = self.angle_diff(angle_to_goal)

            linear_speed = min(self.Kp_lin * distance, self.linear_speed_limit)
            angular_speed = max(min(self.Kp_ang * diff_angle, self.angular_speed_limit), -self.angular_speed_limit)

            if abs(diff_angle) > 0.3:
                linear_speed = 0.0

            if self.is_stuck(linear_speed):
                if self.recovery_attempts < self.MAX_RECOVERY_ATTEMPTS:
                    self.perform_recovery()
                    continue
                else:
                    rospy.logerr("❌ Múltiples intentos de recuperación fallidos.")
                    self.cmd_vel_pub.publish(Twist())
                    break

            cmd = Twist()
            cmd.linear.x = linear_speed
            cmd.angular.z = angular_speed
            self.cmd_vel_pub.publish(cmd)

            if distance < self.waypoint_tolerance and abs(diff_angle) < self.angle_tolerance:
                rospy.loginfo(f"📍 Waypoint alcanzado: {self.waypoints[self.current_waypoint_index]}")
                self.current_waypoint_index += 1
                self.recovery_attempts = 0
                rospy.sleep(0.5)

            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator3D()
        navigator.navigate()
    except rospy.ROSInterruptException:
        pass
