#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

def quaternion_to_yaw(qx, qy, qz, qw):
    """Converts a quaternion into yaw (z-axis rotation)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    """Normalize angle to be within the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class EbotNavigator(Node):
    def __init__(self):
        super().__init__('ebot_nav')

        # --- QOS Profile for Reliable, Non-Latching Communication ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # --- Robot State ---
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.start_x, self.start_y = None, None
        self.laser_ranges = []
        self.odom_received = False

        # --- Waypoints & Tolerances ---
        self.waypoints = [
            (-1.53, -1.95,  1.57),
            (0.13,  1.24,  0.00),
            (0.38, -3.20, -1.57) # P3 Pulled back slightly
        ]
        self.pos_tolerance = 0.2
        self.yaw_tolerance = math.radians(6.0)

        # --- EXTREME PERFORMANCE GAINS (To match the e-Yantra video EXACTLY) ---
        self.k_linear = 2.0         # Very high gain for max acceleration
        self.k_angular = 3.0        # Very high gain for sharp in-motion turns
        self.k_rotate = 3.5         # Very high gain for extremely fast stationary rotation
        self.max_lin_speed = 0.6    # High top speed
        self.max_rot_speed = 2.8    # High rotational speed

        # --- Obstacle Avoidance: HARD STOP ONLY ---
        self.hard_stop_dist = 0.30     # Distance to stop for a direct frontal obstacle

        # --- State Machine ---
        self.current_idx = 0
        self.state = 'INITIAL_ADVANCE'

        # --- Control Loop Timer ---
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz

        self.get_logger().info('Ebot navigator node started with FINAL PERFORMANCE REPLICATION logic.')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x, self.start_y = self.x, self.y
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def lidar_callback(self, msg: LaserScan):
        self.laser_ranges = msg.ranges

    def control_loop(self):
        if not self.odom_received or not self.laser_ranges or self.start_x is None:
            return

        if self.state == 'DONE': return
        if self.current_idx >= len(self.waypoints):
            if self.state != 'DONE':
                self.get_logger().info("🎉 All waypoints reached. Task complete.")
                self.stop_robot()
                self.state = 'DONE'
                self.create_timer(1.0, self.shutdown_node)
            return

        cmd = Twist()
        goal_x, goal_y, goal_yaw = self.waypoints[self.current_idx]

        # --- INITIAL STATE to clear the first corner ---
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
        
        # --- NORMAL NAVIGATION STATE MACHINE ---
        distance_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)
        angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
        angle_error = normalize_angle(angle_to_goal - self.yaw)
        final_yaw_error = normalize_angle(goal_yaw - self.yaw)

        # --- Logic for each state ---
        if self.state == 'ROTATE_TO_GOAL':
            # Extremely strict alignment for point-and-shoot accuracy
            if abs(angle_error) > math.radians(2.0):
                cmd.angular.z = self.k_rotate * angle_error
            else:
                self.stop_robot()
                self.state = 'MOVE_TO_GOAL'
                self.get_logger().info(f"➡️ Aligned. Moving to waypoint {self.current_idx+1}.")
        
        elif self.state == 'MOVE_TO_GOAL':
            front_dist = self.get_frontal_distance()

            # HARD STOP safety rule. This is the ONLY obstacle avoidance.
            if front_dist < self.hard_stop_dist:
                self.get_logger().error(f'🛑 HARD STOP! Obstacle at {front_dist:.2f}m.')
                self.stop_robot() # Stop all motion
            
            # Normal forward motion
            else:
                if distance_to_goal > self.pos_tolerance:
                    cmd.linear.x = self.k_linear * distance_to_goal
                    cmd.angular.z = self.k_angular * angle_error
                else: # Reached goal position
                    self.stop_robot()
                    self.state = 'ROTATE_FINAL'
                    self.get_logger().info(f"📍 Position reached. Aligning to final yaw.")

        elif self.state == 'ROTATE_FINAL':
            if abs(final_yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.k_rotate * final_yaw_error
            else:
                self.get_logger().info(f"✅ Waypoint {self.current_idx+1} achieved.")
                self.stop_robot()
                time.sleep(0.2) # Minimal pause
                self.current_idx += 1
                self.state = 'ROTATE_TO_GOAL' if self.current_idx < len(self.waypoints) else 'DONE'

        # Clamp speeds and publish
        cmd.linear.x = max(0.0, min(self.max_lin_speed, cmd.linear.x))
        cmd.angular.z = max(-self.max_rot_speed, min(self.max_rot_speed, cmd.angular.z))
        self.cmd_pub.publish(cmd)
        
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_frontal_distance(self):
        """Gets minimum distance in a narrow cone directly in front."""
        if not self.laser_ranges: return 100.0
        num_readings = len(self.laser_ranges)
        # Narrow cone: -15 to +15 degrees
        arc_size = int(num_readings / 24) # 15 degrees
        front_arc = self.laser_ranges[-arc_size:] + self.laser_ranges[:arc_size]
        valid_readings = [r for r in front_arc if r > 0.0 and not (math.isinf(r) or math.isnan(r))]
        return min(valid_readings) if valid_readings else 100.0
    
    def shutdown_node(self):
        self.get_logger().info('Shutting down Ebot navigator node.')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = EbotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, stopping node.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()

if __name__ == '__main__':
    main()

