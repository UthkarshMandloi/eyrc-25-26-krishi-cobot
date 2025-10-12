#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

def quaternion_to_yaw(qx, qy, qz, qw):
    # standard conversion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class EbotNavigator(Node):
    def __init__(self):
        super().__init__('ebot_nav')

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.laser = None

        # Waypoints: each waypoint is (x, y, yaw)
        self.waypoints = [
            (-1.53, -1.95,  1.57),   # P1
            (0.13,  1.24,  0.00),    # P2
            (0.38, -3.32, -1.57)     # P3
        ]
        # Tolerances
        self.pos_tolerance = 0.3    # meters
        self.yaw_tolerance = math.radians(10.0)  # radians

        # Control gains (tune if needed)
        self.k_linear = 0.4    # proportional gain for linear velocity
        self.k_angular = 1.5   # proportional gain for angular velocity (during motion)
        self.k_rotate = 1.8    # angular gain for rotation-only moves

        # Max speeds (safety)
        self.max_lin_speed = 0.35
        self.max_rot_speed = 1.2

        # Obstacle avoidance params
        self.obstacle_distance_min = 0.45  # meters; if any range in front < this, we avoid
        self.avoid_turn_speed = 0.6

        # Control state machine
        self.current_idx = 0
        self.state = 'ROTATE_TO_GOAL'  # ROTATE_TO_GOAL -> MOVE_TO_GOAL -> ROTATE_FINAL -> DONE

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Ebot navigator node started.')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def lidar_callback(self, msg: LaserScan):
        self.laser = msg

    def control_loop(self):
        # If no waypoints left, stop and return
        if self.current_idx >= len(self.waypoints):
            self.stop_robot()
            if self.state != 'DONE':
                self.get_logger().info("🎉 All waypoints reached. DONE.")
                self.state = 'DONE'
            return

        # must have valid odom and laser (laser optional but helpful)
        goal_x, goal_y, goal_yaw = self.waypoints[self.current_idx]

        # compute errors
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - self.yaw)
        final_yaw_error = normalize_angle(goal_yaw - self.yaw)

        cmd = Twist()

        # If obstacle in front, do avoidance (higher priority)
        if self.laser is not None:
            if self.is_obstacle_in_front(self.obstacle_distance_min):
                # Simple avoidance: rotate toward the clearer side
                left_avg, right_avg = self.left_right_avg_distances()
                if left_avg > right_avg:
                    cmd.angular.z = self.avoid_turn_speed
                else:
                    cmd.angular.z = -self.avoid_turn_speed
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info_once('⚠️ Obstacle detected: performing avoidance turn')
                return

        # State machine for waypoint navigation
        if self.state == 'ROTATE_TO_GOAL':
            # rotate so robot faces the waypoint
            if abs(angle_error) > math.radians(6.0):  # rotate until approximately aligned
                cmd.angular.z = max(-self.max_rot_speed, min(self.max_rot_speed, self.k_rotate * angle_error))
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                return
            else:
                # small alignment done -> switch to move
                self.state = 'MOVE_TO_GOAL'
                self.get_logger().info(f"➡️ Moving toward waypoint {self.current_idx+1}")

        if self.state == 'MOVE_TO_GOAL':
            # if far from goal, move forward with angular correction
            if distance > self.pos_tolerance:
                linear_speed = min(self.max_lin_speed, self.k_linear * distance)
                angular_speed = max(-self.max_rot_speed, min(self.max_rot_speed, self.k_angular * angle_error))
                # if angle error too large, prefer rotating in place
                if abs(angle_error) > math.radians(30.0):
                    linear_speed = 0.0
                cmd.linear.x = linear_speed
                cmd.angular.z = angular_speed
                self.cmd_pub.publish(cmd)
                return
            else:
                # reached position -> rotate to required final yaw
                self.state = 'ROTATE_FINAL'
                self.get_logger().info(f"📍 Reached position for waypoint {self.current_idx+1}, aligning yaw")

        if self.state == 'ROTATE_FINAL':
            if abs(final_yaw_error) > self.yaw_tolerance:
                cmd.angular.z = max(-self.max_rot_speed, min(self.max_rot_speed, self.k_rotate * final_yaw_error))
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                return
            else:
                self.get_logger().info(f"✅ Waypoint {self.current_idx+1} achieved.")
                self.current_idx += 1
                # prepare for next waypoint
                self.state = 'ROTATE_TO_GOAL'
                # small pause
                self.stop_robot()
                time.sleep(0.2)
                return

    def stop_robot(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def is_obstacle_in_front(self, threshold):
        """Return True if any scan point in front sector is less than threshold."""
        if self.laser is None:
            return False
        msg = self.laser
        # Front sector: +/- 20 degrees
        half_angle = math.radians(20.0)
        i_min = int((0 - msg.angle_min) / msg.angle_increment - (half_angle / msg.angle_increment))
        i_max = int((0 - msg.angle_min) / msg.angle_increment + (half_angle / msg.angle_increment))
        i_min = max(0, i_min)
        i_max = min(len(msg.ranges)-1, i_max)
        try:
            front_ranges = [r for r in msg.ranges[i_min:i_max+1] if not math.isinf(r) and not math.isnan(r)]
        except Exception:
            return False
        if not front_ranges:
            return False
        if min(front_ranges) < threshold:
            return True
        return False

    def left_right_avg_distances(self):
        """Return average distances left and right (useful to pick turn direction)."""
        if self.laser is None:
            return 0.0, 0.0
        msg = self.laser
        mid = len(msg.ranges) // 2
        left = [r for r in msg.ranges[mid+10: min(len(msg.ranges), mid+70)] if not math.isinf(r) and not math.isnan(r)]
        right = [r for r in msg.ranges[max(0, mid-70): mid-10] if not math.isinf(r) and not math.isnan(r)]
        left_avg = sum(left)/len(left) if left else 0.0
        right_avg = sum(right)/len(right) if right else 0.0
        return left_avg, right_avg

def main(args=None):
    rclpy.init(args=args)
    node = EbotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, stopping node.')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
