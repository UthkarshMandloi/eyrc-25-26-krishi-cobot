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
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class EbotNavigator(Node):
    def _init_(self):
        super()._init_('ebot_nav')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.laser_ranges = []
        self.lidar_angle_min, self.lidar_angle_increment = 0.0, 0.0
        self.odom_received, self.lidar_received = False, False

        # ### --- THE NEW TRAJECTORY WITH THE ORIGIN FOR P3 ALIGNMENT --- ###
        # Calculate the perfect yaw to face P3 from the Origin
        p3_x, p3_y = 0.38, -3.32
        yaw_origin_to_p3 = math.atan2(p3_y - 0.0, p3_x - 0.0) 
        self.waypoints = [
            (-1.53, -1.95, 1.57),          # P1
            (0.13,   1.24, 0.00),          # P2
            (0.0,    0.0,  yaw_origin_to_p3), # The Origin, with a calculated yaw to perfectly face P3
            (p3_x,   p3_y, -1.57)           # P3
        ]
        self.pos_tolerance, self.yaw_tolerance = 0.15, math.radians(5.0)

        # High performance speeds
        self.k_linear, self.k_angular, self.k_rotate = 2.0, 3.0, 3.5
        self.max_lin_speed, self.max_rot_speed = 2.0, 2.0
        
        self.safety_stop_dist, self.slowdown_dist = 0.30, 0.7
        self.side_safety_dist, self.nudge_gain = 0.28, 2.0
        
        self.final_push_stop_dist = 0.2

        self.current_idx = 0
        self.state = 'ROTATE_TO_GOAL'
        self.get_logger().info('FINAL CODE with ORIGIN ALIGNMENT & FULL LiDAR: INITIALIZED.')
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation; self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def lidar_callback(self, msg: LaserScan):
        self.laser_ranges, self.lidar_angle_min, self.lidar_angle_increment = msg.ranges, msg.angle_min, msg.angle_increment
        self.lidar_received = True

    def get_min_dist_in_arc(self, start_angle_deg, end_angle_deg):
        if not self.lidar_received or not self.laser_ranges: return 100.0
        num_readings = len(self.laser_ranges); center_index = num_readings // 2
        start_index = (center_index + int(math.radians(start_angle_deg) / self.lidar_angle_increment)) % num_readings
        end_index = (center_index + int(math.radians(end_angle_deg) / self.lidar_angle_increment)) % num_readings
        if start_index > end_index: arc = self.laser_ranges[start_index:] + self.laser_ranges[:end_index+1]
        else: arc = self.laser_ranges[start_index:end_index+1]
        valid_readings = [r for r in arc if r > 0.1 and not (math.isinf(r) or math.isnan(r))]
        return min(valid_readings) if valid_readings else 100.0

    def get_obstacle_modifiers(self):
        front_dist = self.get_min_dist_in_arc(-15, 15); left_dist = self.get_min_dist_in_arc(35, 65); right_dist = self.get_min_dist_in_arc(-65, -35)
        linear_scale = 1.0
        if front_dist < self.safety_stop_dist: linear_scale = 0.0
        elif front_dist < self.slowdown_dist: linear_scale = (front_dist - self.safety_stop_dist) / (self.slowdown_dist - self.safety_stop_dist)
        angular_nudge = 0.0
        if left_dist < self.side_safety_dist: angular_nudge = -self.nudge_gain * (self.side_safety_dist - left_dist)
        elif right_dist < self.side_safety_dist: angular_nudge = self.nudge_gain * (self.side_safety_dist - right_dist)
        return linear_scale, angular_nudge

    def control_loop(self):
        if not self.odom_received or not self.lidar_received: return
        if self.state == 'DONE': return
        
        if self.state == 'FINAL_PUSH_SAFE':
            front_dist = self.get_min_dist_in_arc(-5, 5) 
            if front_dist > self.final_push_stop_dist:
                cmd = Twist(); cmd.linear.x = 0.15; self.cmd_pub.publish(cmd)
            else:
                self.get_logger().info(f"BAN BANG! Mission Complete. Wall at {front_dist:.2f}m.")
                self.stop_robot(); self.state = 'DONE'
            return

        cmd = Twist()
        cmd = Twist()
        goal_x, goal_y, goal_yaw = self.waypoints[self.current_idx]
        distance_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)
        world_angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
        angle_to_goal = normalize_angle(world_angle_to_goal - self.yaw)
        final_yaw_error = normalize_angle(goal_yaw - self.yaw)

        if self.state == 'ROTATE_TO_GOAL' or self.state == 'ROTATE_FINAL':
            target_angle_error = angle_to_goal if self.state == 'ROTATE_TO_GOAL' else final_yaw_error
            if abs(target_angle_error) > self.yaw_tolerance: cmd.angular.z = self.k_rotate * target_angle_error
            else:
                self.stop_robot()
                if self.state == 'ROTATE_TO_GOAL':
                    self.state = 'MOVE_TO_GOAL'
                    self.get_logger().info(f"➡️ Aligned. Moving to WP-{self.current_idx + 1} ({goal_x:.2f}, {goal_y:.2f}).")
                else: # ROTATE_FINAL is complete
                    self.get_logger().info(f"✅ Waypoint {self.current_idx + 1} achieved.")
                    time.sleep(0.2)
                    self.current_idx += 1
                    if self.current_idx >= len(self.waypoints): self.state = 'FINAL_PUSH_SAFE' 
                    else: self.state = 'ROTATE_TO_GOAL'
        
        elif self.state == 'MOVE_TO_GOAL':
            if distance_to_goal > self.pos_tolerance:
                # ### --- THIS IS THE KEY FIX - OBSTACLE AVOIDANCE REACTIVATED --- ###
                # The robot now steers toward the goal AND nudges away from walls simultaneously.
                base_linear_speed = self.k_linear * distance_to_goal
                base_angular_speed = self.k_angular * angle_to_goal
                linear_scale, angular_nudge = self.get_obstacle_modifiers()
                cmd.linear.x = base_linear_speed * linear_scale
                cmd.angular.z = base_angular_speed + angular_nudge
            else:
                self.stop_robot(); self.state = 'ROTATE_FINAL'
                self.get_logger().info(f"📍 Position reached for WP-{self.current_idx+1}. Aligning.")

        cmd.linear.x = max(0.0, min(self.max_lin_speed, cmd.linear.x))
        cmd.angular.z = max(-self.max_rot_speed, min(self.max_rot_speed, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    def stop_robot(self): self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = EbotNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info('Keyboard interrupt, stopping node.')
    finally:
        if rclpy.ok(): node.stop_robot(); node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__': main()