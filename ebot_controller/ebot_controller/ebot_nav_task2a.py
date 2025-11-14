#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
import math
import time
from enum import Enum, auto # Import Enum

# --- Helper Functions ---
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

# OPTIMIZED: Define states using an Enum for safety and readability
class State(Enum):
    ROTATE_TO_GOAL = auto()
    MOVE_TO_GOAL = auto()
    ROTATE_FINAL = auto()
    STOPPED_FOR_DETECTION = auto()
    DOCKING_AT_P1 = auto()
    DONE = auto()

class EbotNavTask2A(Node):
    
    def __init__(self):
        super().__init__('ebot_nav_task2a')

        # OPTIMIZED: Group all parameters and "magic numbers" here for easy tuning
        # --- Task Parameters ---
        self.TIMER_PERIOD = 0.05  # 20 Hz control loop
        self.PAUSE_DURATION_SEC = 2.0
        self.WAYPOINT_PAUSE_SEC = 0.5 # Your time.sleep(0.5)
        
        # --- Waypoints & Tolerances ---
        self.waypoints = [
            (0.55, -5.57, 1.57),   # Waypoint 1 (Custom Align) - idx 0
            (0.26, -1.95, 1.57),   # Waypoint 2 (P1 - Dock) - idx 1
            (0.13,  1.24,  0.00),  # Waypoint 3 (Custom) - idx 2
            (-1.53, 1.24,  1.57),  # Waypoint 4 (Custom) - idx 3
            (-1.48, -0.67, -1.57), # Waypoint 5 (P2) - idx 4
            (-1.53, -6.61, -1.57)  # Waypoint 6 (P3 - End) - idx 5
        ]
        self.POS_TOLERANCE = 0.2
        self.YAW_TOLERANCE_RAD = math.radians(10.0)

        # --- Control Gains ---
        self.K_LINEAR = 2.0
        self.K_ANGULAR = 3.0
        self.K_ROTATE = 3.5
        self.K_ROTATE_GENTLE = 1.2
        self.MAX_LIN_SPEED = 0.7
        self.MAX_ROT_SPEED = 3.0

        # --- Obstacle Avoidance Params ---
        self.SAFETY_STOP_DIST = 0.30
        self.SLOWDOWN_DIST = 0.7
        self.SIDE_SAFETY_DIST = 0.28
        self.NUDGE_GAIN = 2.0

        # --- QoS Profile ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/detection_status', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)
        self.create_subscription(String, '/found_shape', self.shape_found_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Robot State Variables ---
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.laser_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        self.odom_received = False
        self.lidar_received = False

        # --- State Machine ---
        self.current_idx = 0
        self.state = State.ROTATE_TO_GOAL # OPTIMIZED: Use Enum
        self.shape_to_process = None
        self.pause_start_time = None
        
        self.get_logger().info('Ebot Task 2A Navigator Node Started (Optimized).')
        
        # --- Control Loop Timer ---
        self.timer = self.create_timer(self.TIMER_PERIOD, self.control_loop)

    # --- Callbacks ---
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def lidar_callback(self, msg: LaserScan):
        self.laser_ranges = msg.ranges
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment
        self.lidar_received = True

    # OPTIMIZED: Callback now *only* sets the state. The control_loop handles the action.
    def shape_found_callback(self, msg: String):
        """Callback for the /found_shape topic. Just sets the state."""
        if self.state == State.MOVE_TO_GOAL and self.shape_to_process is None:
            self.get_logger().info(f"Shape detector found: {msg.data}. Pausing.")
            self.shape_to_process = msg.data  # Store the shape type
            self.state = State.STOPPED_FOR_DETECTION # Set state, let control_loop act

    # --- Helper Methods ---
    def broadcast_current_waypoint_tf(self):
        if self.current_idx >= len(self.waypoints):
            return
        goal_x, goal_y, _ = self.waypoints[self.current_idx]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'waypoint_goal'
        t.transform.translation.x = goal_x
        t.transform.translation.y = goal_y
        self.tf_broadcaster.sendTransform(t)

    # OPTIMIZED: get_min_dist is now a class method, not a nested function
    def get_min_dist(self, start_angle_deg, end_angle_deg):
        """Helper to get min distance from a LiDAR sector."""
        if not self.lidar_received or not self.laser_ranges:
            return 100.0
        
        num_readings = len(self.laser_ranges)
        start_index = int(math.radians(start_angle_deg) / self.lidar_angle_increment) + num_readings // 2
        end_index = int(math.radians(end_angle_deg) / self.lidar_angle_increment) + num_readings // 2
        
        start_index = max(0, min(num_readings - 1, start_index))
        end_index = max(0, min(num_readings - 1, end_index))
        
        if start_index > end_index: 
            start_index, end_index = end_index, start_index
            
        arc = self.laser_ranges[start_index:end_index+1]
        valid_readings = [r for r in arc if r > 0.1 and not (math.isinf(r) or math.isnan(r))]
        return min(valid_readings) if valid_readings else 100.0

    def get_obstacle_modifiers(self):
        """Calculates speed modifiers based on LiDAR obstacle detection."""
        if not self.lidar_received:
            return 1.0, 0.0 # No data, proceed with caution (or full speed)

        front_dist = self.get_min_dist(-15, 15)
        left_dist = self.get_min_dist(35, 65)
        right_dist = self.get_min_dist(-65, -35)

        linear_scale = 1.0
        if front_dist < self.SAFETY_STOP_DIST:
            linear_scale = 0.0
        elif front_dist < self.SLOWDOWN_DIST:
            linear_scale = (front_dist - self.SAFETY_STOP_DIST) / (self.SLOWDOWN_DIST - self.SAFETY_STOP_DIST)

        angular_nudge = 0.0
        if left_dist < self.SIDE_SAFETY_DIST:
            error = self.SIDE_SAFETY_DIST - left_dist
            angular_nudge = -self.NUDGE_GAIN * error
        elif right_dist < self.SIDE_SAFETY_DIST:
            error = self.SIDE_SAFETY_DIST - right_dist
            angular_nudge = self.NUDGE_GAIN * error
            
        return linear_scale, angular_nudge

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # --- Main Control Loop ---
    def control_loop(self):
        if not self.odom_received or not self.lidar_received:
            self.get_logger().warn("Waiting for odom or lidar...", throttle_duration_sec=5.0)
            return

        self.broadcast_current_waypoint_tf()

        # --- Handle Completion ---
        if self.state == State.DONE:
            self.stop_robot()
            return
        
        if self.current_idx >= len(self.waypoints):
            if self.state != State.DONE:
                self.get_logger().info("🎉 All waypoints reached. Mission Accomplished.")
                self.stop_robot()
                self.state = State.DONE
            return

        cmd = Twist() # Initialize Twist message

        # --- State Machine Logic ---

        if self.state == State.STOPPED_FOR_DETECTION:
            if self.pause_start_time is None:
                # This is the first loop tick in this state. Take action.
                self.stop_robot()
                report = String()
                report.data = f"{self.shape_to_process},{self.x:.2f},{self.y:.2f}"
                self.status_pub.publish(report)
                self.get_logger().info(f"Published: {report.data}. Waiting {self.PAUSE_DURATION_SEC}s.")
                self.pause_start_time = self.get_clock().now()

            # This is the waiting logic
            elapsed_time = (self.get_clock().now() - self.pause_start_time).nanoseconds / 1e9
            if elapsed_time < self.PAUSE_DURATION_SEC:
                return # Keep waiting, don't publish cmd_vel
            else:
                self.get_logger().info("Resume navigation.")
                self.shape_to_process = None
                self.pause_start_time = None
                self.state = State.MOVE_TO_GOAL # Go back to moving

        elif self.state == State.DOCKING_AT_P1:
            # This is the waiting logic for the dock
            elapsed_time = (self.get_clock().now() - self.pause_start_time).nanoseconds / 1e9
            if elapsed_time < self.PAUSE_DURATION_SEC:
                return # Keep waiting, don't publish cmd_vel
            else:
                self.get_logger().info("2s dock complete. Moving to next waypoint.")
                self.current_idx += 1
                self.state = State.ROTATE_TO_GOAL
                self.pause_start_time = None

        elif self.state == State.ROTATE_TO_GOAL or self.state == State.ROTATE_FINAL:
            goal_x, goal_y, goal_yaw = self.waypoints[self.current_idx]
            world_angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
            angle_to_goal = normalize_angle(world_angle_to_goal - self.yaw)
            final_yaw_error = normalize_angle(goal_yaw - self.yaw)
            
            target_angle_error = angle_to_goal if self.state == State.ROTATE_TO_GOAL else final_yaw_error
            
            if abs(target_angle_error) > self.YAW_TOLERANCE_RAD:
                if self.state == State.ROTATE_FINAL and self.current_idx == 2: # Waypoint 3
                    cmd.angular.z = self.K_ROTATE_GENTLE * target_angle_error
                else:
                    cmd.angular.z = self.K_ROTATE * target_angle_error
            else:
                # --- Angle is aligned, decide next state ---
                self.stop_robot()
                if self.state == State.ROTATE_TO_GOAL:
                    self.state = State.MOVE_TO_GOAL
                
                else: # We were in ROTATE_FINAL, meaning waypoint is complete
                    self.get_logger().info(f"✅ Waypoint {self.current_idx + 1} fully achieved.")
                    
                    if self.current_idx == 1:
                        # This is Waypoint 2 (P1 Dock), begin mandatory 2s stop
                        self.get_logger().info("At Dock (P1). Publishing status and waiting 2s.")
                        report = String()
                        report.data = f"DOCK_STATION,{self.waypoints[1][0]},{self.waypoints[1][1]}"
                        self.status_pub.publish(report)
                        
                        self.state = State.DOCKING_AT_P1
                        self.pause_start_time = self.get_clock().now()
                    
                    elif self.current_idx == (len(self.waypoints) - 1):
                        self.get_logger().info("🏁 Final waypoint reached. Task complete.")
                        self.state = State.DONE
                    
                    else:
                        # This should not be reachable due to the MOVE_TO_GOAL optimization
                        # But as a fallback, we'll keep your original logic
                        time.sleep(self.WAYPOINT_PAUSE_SEC)
                        self.current_idx += 1
                        self.state = State.ROTATE_TO_GOAL

        elif self.state == State.MOVE_TO_GOAL:
            goal_x, goal_y, _ = self.waypoints[self.current_idx]
            distance_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)

            if distance_to_goal > self.POS_TOLERANCE:
                world_angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
                angle_to_goal = normalize_angle(world_angle_to_goal - self.yaw)
                
                base_linear_speed = self.K_LINEAR * distance_to_goal
                base_angular_speed = self.K_ANGULAR * angle_to_goal
                
                linear_scale, angular_nudge = self.get_obstacle_modifiers()
                
                cmd.linear.x = base_linear_speed * linear_scale
                cmd.angular.z = base_angular_speed + angular_nudge
            else:
                # --- OPTIMIZATION: Waypoint position reached ---
                self.stop_robot()
                
                # We must perform ROTATE_FINAL for the Dock (idx 1) and the End (last idx, which is 5)
                is_dock_waypoint = (self.current_idx == 1)
                is_final_waypoint = (self.current_idx == (len(self.waypoints) - 1))

                if is_dock_waypoint or is_final_waypoint:
                    # For dock and end, we must do the final alignment
                    self.get_logger().info(f"📍 Position {self.current_idx + 1} reached. Aligning to final yaw.")
                    self.state = State.ROTATE_FINAL
                else:
                    # For all other intermediate points, skip final rotation
                    self.get_logger().info(f"✅ Waypoint {self.current_idx + 1} position reached. Skipping final rotation.")
                    time.sleep(self.WAYPOINT_PAUSE_SEC) # Your 0.5s pause
                    self.current_idx += 1
                    self.state = State.ROTATE_TO_GOAL
        
        # --- Clamp speeds and publish ---
        cmd.linear.x = max(0.0, min(self.MAX_LIN_SPEED, cmd.linear.x))
        cmd.angular.z = max(-self.MAX_ROT_SPEED, min(self.MAX_ROT_SPEED, cmd.angular.z))
        
        # Only publish if we are not in a waiting state
        if self.state not in [State.STOPPED_FOR_DETECTION, State.DOCKING_AT_P1]:
            self.cmd_pub.publish(cmd)

# --- Main execution ---
def main(args=None):
    rclpy.init(args=args)
    node = EbotNavTask2A()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        node.get_logger().info('Keyboard interrupt, stopping node.')
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.try_shutdown()

if __name__ == '__main__':
    main()