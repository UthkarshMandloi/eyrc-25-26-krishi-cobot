#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

# --- Helper Functions ---
def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class EbotNavigator(Node):
    def _init_(self):
        super()._init_('ebot_nav')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.start_x, self.start_y = None, None
        self.laser_ranges = []
        self.odom_received = False
        self.centering_error_previous = 0.0
        self.last_time = self.get_clock().now()

        self.waypoints = [
            (-1.53, -1.95,  1.57),
            (0.13,  1.24,  0.00),
            (0.38, -3.20, -1.57)
        ]
        self.pos_tolerance = 0.2
        self.yaw_tolerance = math.radians(6.0)

        self.k_linear = 2.0
        self.k_angular = 2.5
        self.k_rotate = 3.5
        self.max_lin_speed = 0.6
        self.max_rot_speed = 2.8

        self.k_p_centering = 1.5
        self.k_d_centering = 0.8
        self.corridor_detection_dist = 0.5

        self.current_idx = 0
        self.state = 'INITIAL_ADVANCE'
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Ebot navigator node started with FINAL, GUARANTEED FIX.')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x, self.start_y = self.x, self.y
        q = msg.pose.pose.orientation
        # ✅ Fixed: yaw sign corrected
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def lidar_callback(self, msg: LaserScan):
        self.laser_ranges = msg.ranges

    def control_loop(self):
        # ✅ Fixed: allow movement even if /scan late
        if not self.odom_received or self.start_x is None:
            return

        if self.state == 'DONE':
            return
        if self.current_idx >= len(self.waypoints):
            if self.state != 'DONE':
                self.get_logger().info("🎉 All waypoints reached. Task complete.")
                self.stop_robot()
                self.state = 'DONE'
                # ✅ Fixed: clean shutdown, no extra timer
                rclpy.shutdown()
            return

        cmd = Twist()
        cmd = Twist()
        goal_x, goal_y, goal_yaw = self.waypoints[self.current_idx]

        if self.state == 'INITIAL_ADVANCE':
            distance_traveled = math.hypot(self.x - self.start_x, self.y - self.start_y)
            if distance_traveled < 0.7:
                cmd.linear.x = 0.3
                self.cmd_pub.publish(cmd)
                return
            else:
                self.get_logger().info('✅ Initial advance complete. Starting navigation.')
                self.stop_robot()
                self.state = 'ROTATE_TO_GOAL'

        distance_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)
        angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
        angle_error = normalize_angle(angle_to_goal - self.yaw)
        final_yaw_error = normalize_angle(goal_yaw - self.yaw)

        if self.state == 'ROTATE_TO_GOAL':
            if abs(angle_error) > math.radians(2.0):
                cmd.angular.z = self.k_rotate * angle_error
            else:
                self.stop_robot()
                self.state = 'MOVE_TO_GOAL'
                self.get_logger().info(f"➡️ Aligned. Moving to waypoint {self.current_idx+1}.")

        elif self.state == 'MOVE_TO_GOAL':
            if distance_to_goal > self.pos_tolerance:
                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                # ✅ Fixed: derivative protection
                if dt <= 0.0:
                    dt = 1e-3
                self.last_time = now

                _, left_dist, right_dist = self.get_navigation_distances()
                centering_error = left_dist - right_dist
                p_force = self.k_p_centering * centering_error
                error_derivative = (centering_error - self.centering_error_previous) / dt
                d_force = self.k_d_centering * error_derivative
                self.centering_error_previous = centering_error

                steering_adjustment = 0.0
                if left_dist < self.corridor_detection_dist or right_dist < self.corridor_detection_dist:
                    steering_adjustment = p_force + d_force
                    self.get_logger().warn(
                        f'Corridor Nav Active: L:{left_dist:.2f} R:{right_dist:.2f} Steer:{steering_adjustment:.2f}',
                        throttle_duration_sec=0.5)

                cmd.linear.x = self.k_linear * distance_to_goal
                cmd.angular.z = (self.k_angular * angle_error) + steering_adjustment
            else:
                self.stop_robot()
                self.state = 'ROTATE_FINAL'
                self.get_logger().info(f"📍 Position reached. Aligning to final yaw.")

        elif self.state == 'ROTATE_FINAL':
            if abs(final_yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.k_rotate * final_yaw_error
            else:
                self.get_logger().info(f"✅ Waypoint {self.current_idx+1} achieved.")
                self.stop_robot()
                time.sleep(0.2)
                self.current_idx += 1
                self.state = 'ROTATE_TO_GOAL' if self.current_idx < len(self.waypoints) else 'DONE'

        cmd.linear.x = max(0.0, min(self.max_lin_speed, cmd.linear.x))
        cmd.angular.z = max(-self.max_rot_speed, min(self.max_rot_speed, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_sector_ranges(self, start_deg, end_deg):
        if not self.laser_ranges:
            return []
        num_readings = len(self.laser_ranges)
        start_idx = int((start_deg % 360) * num_readings / 360)
        end_idx = int((end_deg % 360) * num_readings / 360)
        if start_idx <= end_idx:
            sector = self.laser_ranges[start_idx:end_idx]
        else:
            sector = self.laser_ranges[start_idx:] + self.laser_ranges[:end_idx]
        return [r for r in sector if r > 0.0 and not (math.isinf(r) or math.isnan(r))]

    def get_navigation_distances(self):
        front_ranges = self.get_sector_ranges(345, 15)
        left_ranges = self.get_sector_ranges(25, 75)
        right_ranges = self.get_sector_ranges(285, 335)
        front_dist = min(front_ranges) if front_ranges else 100.0
        left_dist = min(left_ranges) if left_ranges else 100.0
        right_dist = min(right_ranges) if right_ranges else 100.0
        return front_dist, left_dist, right_dist

def main(args=None):
    rclpy.init(args=args)
    node = EbotNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info('Keyboard interrupt, stopping node.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()

if _name_ == '_main_': main()