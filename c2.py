#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi
from tf.transformations import euler_from_quaternion
import time

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigation', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.load_parameters()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_positions = []
        self.odom_history = []
        self.oscillation_counter = 0

        self.prev_time = time.time()
        self.is_lifting = False
        self.lift_start_time = None
        self.robot_ready = False
        self.state = "LIFTING"

        self.integral_lin = 0.0
        self.integral_ang = 0.0

        rospy.loginfo("Navegador inicializado.")

    def load_parameters(self):
        self.waypoints = rospy.get_param('~waypoints', [(1, 0), (3, 0), (3.5, -1), (7, -1)])
        self.current_waypoint_index = rospy.get_param('~initial_waypoint_index', 0)

        self.Kp_lin = rospy.get_param('~kp_linear', 0.6)
        self.Ki_lin = rospy.get_param('~ki_linear', 0.05)
        self.integral_lin_max = rospy.get_param('~integral_linear_max', 0.12)

        self.Kp_ang = rospy.get_param('~kp_angular', 3.0)
        self.Ki_ang = rospy.get_param('~ki_angular', 0.1)
        self.integral_ang_max = rospy.get_param('~integral_angular_max', 0.5)

        self.stuck_threshold = rospy.get_param('~stuck_threshold', 0.02)
        self.stuck_steps = rospy.get_param('~stuck_steps', 10)
        self.retro_time = rospy.get_param('~backward_duration', 1.5)
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 0.1)
        self.giro_solo_threshold = rospy.get_param('~rotate_only_threshold', 0.8)
        self.reduccion_velocidad_threshold = rospy.get_param('~speed_reduction_threshold', 0.2)

        self.lift_duration_up = rospy.get_param('~lift_up_duration', 2.0)
        self.lift_wait_duration = rospy.get_param('~lift_wait_duration', 3.0)
        self.backward_speed = rospy.get_param('~backward_speed', -0.15)
        self.linear_speed_min = rospy.get_param('~linear_speed_min', 0.10)
        self.linear_speed_max = rospy.get_param('~linear_speed_max', 0.22)
        self.angular_speed_limit = rospy.get_param('~angular_speed_limit', 0.2)

        rospy.loginfo("Parámetros cargados.")
        rospy.loginfo(f"Waypoints: {self.waypoints}")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.odom_history.append((x, y))
        if len(self.odom_history) > 7:
            self.odom_history.pop(0)

        avg_x = sum([p[0] for p in self.odom_history]) / len(self.odom_history)
        avg_y = sum([p[1] for p in self.odom_history]) / len(self.odom_history)

        self.current_x = avg_x
        self.current_y = avg_y
        self.current_theta = theta

        if len(self.odom_history) >= 3:
            dir1 = self.odom_history[-1][0] - self.odom_history[-2][0]
            dir2 = self.odom_history[-2][0] - self.odom_history[-3][0]
            if dir1 * dir2 < 0:
                self.oscillation_counter += 1
            else:
                self.oscillation_counter = max(0, self.oscillation_counter - 1)

    def lift_robot(self):
        if self.state == "LIFTING":
            if not self.is_lifting:
                rospy.loginfo("Levantando el robot...")
                self.is_lifting = True
                self.lift_start_time = rospy.Time.now()
            elif (rospy.Time.now() - self.lift_start_time).to_sec() < self.lift_duration_up:
                pass
            elif (rospy.Time.now() - self.lift_start_time).to_sec() < (self.lift_duration_up + self.lift_wait_duration):
                rospy.loginfo("Esperando para estabilización...")
            else:
                rospy.loginfo("Robot listo para comenzar la navegación.")
                self.robot_ready = True
                self.is_lifting = False
                self.lift_start_time = None
                self.state = "NAVIGATING"

    def calculate_control(self, target):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        distance = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)
        angle_diff = (angle_to_goal - self.current_theta + pi) % (2 * pi) - pi

        self.integral_ang += angle_diff * dt
        self.integral_ang = max(min(self.integral_ang, self.integral_ang_max), -self.integral_ang_max)
        angular_speed = self.Kp_ang * angle_diff + self.Ki_ang * self.integral_ang
        angular_speed = max(min(angular_speed, self.angular_speed_limit), -self.angular_speed_limit)

        linear_speed = 0.0
        if abs(angle_diff) <= self.giro_solo_threshold:
            self.integral_lin += distance * dt
            self.integral_lin = max(min(self.integral_lin, self.integral_lin_max), 0.0)

            base_linear_speed = self.Kp_lin * distance + self.Ki_lin * self.integral_lin

            # Escalado adaptativo
            linear_speed = max(self.linear_speed_min, min(base_linear_speed, self.linear_speed_max))

            if abs(angle_diff) > self.reduccion_velocidad_threshold:
                linear_speed *= 0.5

        if self.oscillation_counter >= 3:
            rospy.logwarn("Oscilación detectada. Pausando para estabilizar.")
            self.stop_robot()
            rospy.sleep(1.5)
            self.oscillation_counter = 0

        return linear_speed, angular_speed

    def move(self, linear_speed, angular_speed):
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.move(0.0, 0.0)
        rospy.loginfo("Robot detenido.")
        self.integral_lin = 0.0
        self.integral_ang = 0.0

    def distance_to_point(self, p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def detect_stuck(self):
        self.last_positions.append((self.current_x, self.current_y))
        if len(self.last_positions) > self.stuck_steps:
            self.last_positions.pop(0)
            moved = self.distance_to_point(self.last_positions[0], self.last_positions[-1])
            if moved < self.stuck_threshold:
                rospy.logwarn("Robot parece atascado.")
                return True
        return False

    def move_backward(self):
        rospy.logwarn("Retrocediendo...")
        self.move(self.backward_speed, 0.0)
        rospy.sleep(self.retro_time)
        self.stop_robot()
        self.state = "NAVIGATING"

    def navigate(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == "LIFTING":
                self.lift_robot()
            elif self.state == "NAVIGATING":
                if self.current_waypoint_index < len(self.waypoints):
                    target = self.waypoints[self.current_waypoint_index]
                    linear_speed, angular_speed = self.calculate_control(target)
                    self.move(linear_speed, angular_speed)

                    dist = self.distance_to_point((self.current_x, self.current_y), target)
                    if dist < self.waypoint_tolerance:
                        rospy.loginfo(f"Waypoint {target} alcanzado.")
                        self.current_waypoint_index += 1
                        self.stop_robot()

                    if self.detect_stuck():
                        self.state = "BACKWARD"
                else:
                    rospy.loginfo("🟢 Todos los waypoints alcanzados.")
                    self.state = "RETURNING"
            elif self.state == "BACKWARD":
                self.move_backward()
            elif self.state == "RETURNING":
                target = self.waypoints[0]
                linear_speed, angular_speed = self.calculate_control(target)
                self.move(linear_speed, angular_speed)
                if self.distance_to_point((self.current_x, self.current_y), target) < self.waypoint_tolerance:
                    rospy.loginfo("🏁 Regresado al punto inicial.")
                    self.stop_robot()
                    self.state = "IDLE"
                    rospy.signal_shutdown("Navegación completada.")
            elif self.state == "IDLE":
                rospy.sleep(1)
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        pass
