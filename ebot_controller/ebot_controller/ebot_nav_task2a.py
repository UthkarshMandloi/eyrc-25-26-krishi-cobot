#!/usr/bin/env python3
'''
# Team ID:          eYRC#4686
# Theme:            Krishi coBot
# Author List:      <Names of team members worked on this file>
# Filename:         ebot_nav_task2a.py
# Functions:        quaternion_to_yaw, normalize_angle, __init__, odom_callback, 
#                   lidar_callback, shape_found_callback, broadcast_current_waypoint_tf, 
#                   get_min_dist, get_obstacle_modifiers, stop_robot, control_loop, main
# Global variables: None
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
# MODIFIED: Import the correct QoS profile for sensors
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, SensorDataQoSProfile
import tf2_ros
import math
import time
from enum import Enum, auto # Import Enum

# --- Helper Functions ---

def quaternion_to_yaw(qx, qy, qz, qw):
    '''
    Purpose:
    ---
    Converts a quaternion (qx, qy, qz, qw) into a single yaw angle (z-axis rotation).

    Input Arguments:
    ---
    `qx` :  [ float ]
        The x-component of the quaternion.
    `qy` :  [ float ]
        The y-component of the quaternion.
    `qz` :  [ float ]
        The z-component of the quaternion.
    `qw` :  [ float ]
        The w-component of the quaternion.

    Returns:
    ---
    `yaw` :  [ float ]
        The calculated yaw angle in radians.

    Example call:
    ---
    yaw = quaternion_to_yaw(0.0, 0.0, 0.707, 0.707)
    '''
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    '''
    Purpose:
    ---
    Ensures an angle is always within the range [-pi, pi].

    Input Arguments:
    ---
    `angle` :  [ float ]
        The input angle in radians.

    Returns:
    ---
    `angle` :  [ float ]
        The normalized angle in radians.

    Example call:
    ---
    normalized_angle = normalize_angle(4.0)
    '''
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# Define states using an Enum for safety and readability
class State(Enum):
    ROTATE_TO_GOAL = auto()
    MOVE_TO_GOAL = auto()
    ROTATE_FINAL = auto()
    STOPPED_FOR_DETECTION = auto()
    DOCKING_AT_P1 = auto()
    DONE = auto()

class EbotNavTask2A(Node):
    '''
    Purpose:
    ---
    Main node for Task 2A. Controls the eBot's navigation state machine,
    handles waypoint following, obstacle avoidance, and pausing for
    shape detections.
    '''
    
    def __init__(self):
        '''
        Purpose:
        ---
        Initializes the EbotNavTask2A node. Sets up all parameters,
        gains, publishers, subscribers, and the main control loop timer.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        node = EbotNavTask2A()
        '''
        super().__init__('ebot_nav_task2a')

        # --- Task Parameters ---
        # TIMER_PERIOD: Frequency of the main control loop (in seconds)
        self.TIMER_PERIOD = 0.05  # 20 Hz
        # PAUSE_DURATION_SEC: Time to wait after detecting a shape or docking
        self.PAUSE_DURATION_SEC = 2.0
        # WAYPOINT_PAUSE_SEC: Brief pause after reaching an intermediate waypoint
        self.WAYPOINT_PAUSE_SEC = 0.5
        
        # --- Waypoints & Tolerances ---
        # waypoints: The list of (x, y, yaw) coordinates to visit in order
        self.waypoints = [
            (0.55, -5.57, 1.57),   # Waypoint 1 (Custom Align) - idx 0
            (0.26, -1.95, 1.57),   # Waypoint 2 (P1 - Dock) - idx 1
            (0.13,  1.24,  0.00),  # Waypoint 3 (Custom) - idx 2
            (-1.53, 1.24,  1.57),  # Waypoint 4 (Custom) - idx 3
            (-1.48, -0.67, -1.57), # Waypoint 5 (P2) - idx 4
            (-1.53, -6.61, -1.57)  # Waypoint 6 (P3 - End) - idx 5
        ]
        # POS_TOLERANCE: Allowed distance error from a waypoint (meters)
        self.POS_TOLERANCE = 0.2
        # YAW_TOLERANCE_RAD: Allowed angular error from a waypoint (radians)
        self.YAW_TOLERANCE_RAD = math.radians(10.0)

        # --- Control Gains (MODIFIED for higher speed) ---
        self.K_LINEAR = 2.5           # MODIFIED: Increased from 2.0
        self.K_ANGULAR = 3.5          # MODIFIED: Increased from 3.0
        self.K_ROTATE = 4.0           # MODIFIED: Increased from 3.5
        self.K_ROTATE_GENTLE = 1.2
        self.MAX_LIN_SPEED = 1.0      # MODIFIED: Increased from 0.7
        self.MAX_ROT_SPEED = 4.0      # MODIFIED: Increased from 3.0

        # --- Obstacle Avoidance Params ---
        self.SAFETY_STOP_DIST = 0.30
        self.SLOWDOWN_DIST = 0.7
        self.SIDE_SAFETY_DIST = 0.28
        self.NUDGE_GAIN = 2.0

        # --- Publishers & Subscribers ---
        # cmd_pub: Publishes Twist commands to move the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # status_pub: Publishes the official detection status string
        self.status_pub = self.create_publisher(String, '/detection_status', 10)
        
        # Odom and Scan subscribers use a specific QoS for sensor data
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=SensorDataQoSProfile())
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile=SensorDataQoSProfile())
        
        # shape_found_callback: Listens to the internal /found_shape topic
        self.create_subscription(String, '/found_shape', self.shape_found_callback, 10)
        # tf_broadcaster: Publishes the 'waypoint_goal' frame for RViz
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Robot State Variables ---
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.laser_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        self.odom_received = False
        self.lidar_received = False

        # --- State Machine ---
        # current_idx: The index of the waypoint we are currently targeting
        self.current_idx = 0
        # state: The robot's current behavior state
        self.state = State.ROTATE_TO_GOAL
        # shape_to_process: Stores the name of the shape (e.g., "BAD_HEALTH")
        self.shape_to_process = None
        # pause_start_time: Stores the time when a pause (for detection or docking) began
        self.pause_start_time = None
        
        self.get_logger().info('Ebot Task 2A Navigator Node Started (Optimized).')
        
        # --- Control Loop Timer ---
        self.timer = self.create_timer(self.TIMER_PERIOD, self.control_loop)

    # --- Callbacks ---
    def odom_callback(self, msg: Odometry):
        '''
        Purpose:
        ---
        Callback function for the /odom topic. Updates the robot's
        current position (x, y) and yaw angle.
        
        Input Arguments:
        ---
        `msg` :  [ nav_msgs.msg.Odometry ]
            The incoming odometry message.

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically by the odom subscriber.
        '''
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def lidar_callback(self, msg: LaserScan):
        '''
        Purpose:
        ---
        Callback function for the /scan topic. Updates the robot's
        internal knowledge of its LiDAR readings.
        
        Input Arguments:
        ---
        `msg` :  [ sensor_msgs.msg.LaserScan ]
            The incoming LiDAR scan message.

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically by the scan subscriber.
        '''
        self.laser_ranges = msg.ranges
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment
        self.lidar_received = True

    def shape_found_callback(self, msg: String):
        '''
        Purpose:
        ---
        Callback for the internal /found_shape topic.
        If the robot is moving, this function triggers the
        'STOPPED_FOR_DETECTION' state.
        
        Input Arguments:
        ---
        `msg` :  [ std_msgs.msg.String ]
            The message from the shape_detector node (e.g., "BAD_HEALTH").

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically by the /found_shape subscriber.
        '''
        # Only process a new shape if we are currently moving
        if self.state == State.MOVE_TO_GOAL and self.shape_to_process is None:
            self.get_logger().info(f"Shape detector found: {msg.data}. Pausing.")
            self.shape_to_process = msg.data
            self.state = State.STOPPED_FOR_DETECTION

    # --- Helper Methods ---
    def broadcast_current_waypoint_tf(self):
        '''
        Purpose:
        ---
        Publishes the current target waypoint as a TF frame
        named 'waypoint_goal' for visualization in RViz.
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.broadcast_current_waypoint_tf()
        '''
        if self.current_idx >= len(self.waypoints):
            return
        goal_x, goal_y, _ = self.waypoints[self.current_idx]
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'waypoint_goal'
        
        t.transform.translation.x = goal_x
        t.transform.translation.y = goal_y
        t.transform.rotation.w = 1.0 # No rotation
        
        self.tf_broadcaster.sendTransform(t)

    def get_min_dist(self, start_angle_deg, end_angle_deg):
        '''
        Purpose:
        ---
        Helper function to get the minimum valid distance reading
        from a specific angular sector of the LiDAR scan.
        
        Input Arguments:
        ---
        `start_angle_deg` :  [ int ]
            The starting angle of the sector (in degrees).
        `end_angle_deg` :  [ int ]
            The ending angle of the sector (in degrees).

        Returns:
        ---
        `min_distance` :  [ float ]
            The minimum valid distance in that sector.

        Example call:
        ---
        front_dist = self.get_min_dist(-15, 15)
        '''
        if not self.lidar_received or not self.laser_ranges:
            return 100.0
        
        num_readings = len(self.laser_ranges)
        # Calculate the indices in the laser_ranges array
        start_index = int(math.radians(start_angle_deg) / self.lidar_angle_increment) + num_readings // 2
        end_index = int(math.radians(end_angle_deg) / self.lidar_angle_increment) + num_readings // 2
        
        start_index = max(0, min(num_readings - 1, start_index))
        end_index = max(0, min(num_readings - 1, end_index))
        
        if start_index > end_index: 
            start_index, end_index = end_index, start_index
            
        arc = self.laser_ranges[start_index:end_index+1]
        # Filter out invalid readings (0, inf, nan)
        valid_readings = [r for r in arc if r > 0.1 and not (math.isinf(r) or math.isnan(r))]
        
        return min(valid_readings) if valid_readings else 100.0

    def get_obstacle_modifiers(self):
        '''
        Purpose:
        ---
        Implements a basic obstacle avoidance algorithm.
        It calculates a linear speed scale (0.0 to 1.0) and an
        angular "nudge" to avoid obstacles.
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        `linear_scale` :  [ float ]
            A multiplier for linear speed (0.0 = stop, 1.0 = full speed).
        `angular_nudge` :  [ float ]
            An angular velocity to add to the robot to "nudge" it away from side obstacles.

        Example call:
        ---
        lin_scale, ang_nudge = self.get_obstacle_modifiers()
        '''
        if not self.lidar_received:
            return 1.0, 0.0

        # Get distances from front, left, and right sectors
        front_dist = self.get_min_dist(-15, 15)
        left_dist = self.get_min_dist(35, 65)
        right_dist = self.get_min_dist(-65, -35)

        # Calculate linear slowdown/stop
        linear_scale = 1.0
        if front_dist < self.SAFETY_STOP_DIST:
            linear_scale = 0.0  # Stop
        elif front_dist < self.SLOWDOWN_DIST:
            # Scale speed proportionally between safety and slowdown distances
            linear_scale = (front_dist - self.SAFETY_STOP_DIST) / (self.SLOWDOWN_DIST - self.SAFETY_STOP_DIST)

        # Calculate angular nudge
        angular_nudge = 0.0
        if left_dist < self.SIDE_SAFETY_DIST:
            error = self.SIDE_SAFETY_DIST - left_dist
            angular_nudge = -self.NUDGE_GAIN * error # Nudge right
        elif right_dist < self.SIDE_SAFETY_DIST:
            error = self.SIDE_SAFETY_DIST - right_dist
            angular_nudge = self.NUDGE_GAIN * error # Nudge left
            
        return linear_scale, angular_nudge

    def stop_robot(self):
        '''
        Purpose:
        ---
        Publishes an empty Twist message (all zeros) to
        stop the robot.
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.stop_robot()
        '''
        self.cmd_pub.publish(Twist())

    # --- Main Control Loop ---
    def control_loop(self):
        '''
        Purpose:
        ---
        The main state machine for the robot. This function is called
        at 20Hz and determines what the robot should be doing
        (rotating, moving, pausing, etc.)
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically by the self.timer.
        '''
        
        # Wait until we have sensor data
        if not self.odom_received or not self.lidar_received:
            self.get_logger().warn("Waiting for odom or lidar...", throttle_duration_sec=5.0)
            return

        self.broadcast_current_waypoint_tf()

        # --- Handle Completion ---
        if self.state == State.DONE:
            self.stop_robot()
            return
        
        # Check if we have finished all waypoints
        if self.current_idx >= len(self.waypoints):
            if self.state != State.DONE:
                self.get_logger().info("🎉 All waypoints reached. Mission Accomplished.")
                self.stop_robot()
                self.state = State.DONE
            return

        cmd = Twist() # Initialize Twist message

        # --- State: STOPPED_FOR_DETECTION ---
        if self.state == State.STOPPED_FOR_DETECTION:
            if self.pause_start_time is None:
                # First loop tick in this state: Stop, publish, and start timer
                self.stop_robot()
                report = String()
                report.data = f"{self.shape_to_process},{self.x:.2f},{self.y:.2f}"
                self.status_pub.publish(report)
                self.get_logger().info(f"Published: {report.data}. Waiting {self.PAUSE_DURATION_SEC}s.")
                self.pause_start_time = self.get_clock().now()

            # Wait for pause to finish
            elapsed_time = (self.get_clock().now() - self.pause_start_time).nanoseconds / 1e9
            if elapsed_time < self.PAUSE_DURATION_SEC:
                return # Keep waiting
            else:
                # Pause finished, resume navigation
                self.get_logger().info("Resume navigation.")
                self.shape_to_process = None
                self.pause_start_time = None
                self.state = State.MOVE_TO_GOAL

        # --- State: DOCKING_AT_P1 ---
        elif self.state == State.DOCKING_AT_P1:
            # Wait for 2-second mandatory dock
            elapsed_time = (self.get_clock().now() - self.pause_start_time).nanoseconds / 1e9
            if elapsed_time < self.PAUSE_DURATION_SEC:
                return # Keep waiting
            else:
                # Docking finished, move to the next waypoint
                self.get_logger().info("2s dock complete. Moving to next waypoint.")
                self.current_idx += 1
                self.state = State.ROTATE_TO_GOAL
                self.pause_start_time = None

        # --- State: ROTATE_TO_GOAL or ROTATE_FINAL ---
        elif self.state == State.ROTATE_TO_GOAL or self.state == State.ROTATE_FINAL:
            # Get target goal
            goal_x, goal_y, goal_yaw = self.waypoints[self.current_idx]
            
            # Calculate errors
            world_angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
            angle_to_goal = normalize_angle(world_angle_to_goal - self.yaw)
            final_yaw_error = normalize_angle(goal_yaw - self.yaw)
            
            target_angle_error = angle_to_goal if self.state == State.ROTATE_TO_GOAL else final_yaw_error
            
            # P-controller for rotation
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
                else: 
                    # We were in ROTATE_FINAL, meaning waypoint is complete
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
                        # This is the FINAL waypoint
                        self.get_logger().info("🏁 Final waypoint reached. Task complete.")
                        self.state = State.DONE
                    
                    else:
                        # This fallback is for the (now unused) gentle pause
                        time.sleep(self.WAYPOINT_PAUSE_SEC)
                        self.current_idx += 1
                        self.state = State.ROTATE_TO_GOAL

        # --- State: MOVE_TO_GOAL ---
        elif self.state == State.MOVE_TO_GOAL:
            goal_x, goal_y, _ = self.waypoints[self.current_idx]
            distance_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)

            # P-controller for linear movement
            if distance_to_goal > self.POS_TOLERANCE:
                world_angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
                angle_to_goal = normalize_angle(world_angle_to_goal - self.yaw)
                
                base_linear_speed = self.K_LINEAR * distance_to_goal
                base_angular_speed = self.K_ANGULAR * angle_to_goal
                
                # Get obstacle avoidance modifiers
                linear_scale, angular_nudge = self.get_obstacle_modifiers()
                
                cmd.linear.x = base_linear_speed * linear_scale
                cmd.angular.z = base_angular_speed + angular_nudge
            else:
                # --- Waypoint position reached ---
                self.stop_robot()
                
                is_dock_waypoint = (self.current_idx == 1)
                is_final_waypoint = (self.current_idx == (len(self.waypoints) - 1))

                # Optimization: Only do final rotation for dock and end
                if is_dock_waypoint or is_final_waypoint:
                    self.get_logger().info(f"📍 Position {self.current_idx + 1} reached. Aligning to final yaw.")
                    self.state = State.ROTATE_FINAL
                else:
                    # Skip final rotation for intermediate points
                    self.get_logger().info(f"✅ Waypoint {self.current_idx + 1} position reached. Skipping final rotation.")
                    time.sleep(self.WAYPOINT_PAUSE_SEC)
                    self.current_idx += 1
                    self.state = State.ROTATE_TO_GOAL
        
        # --- Clamp speeds to max values ---
        cmd.linear.x = max(0.0, min(self.MAX_LIN_SPEED, cmd.linear.x))
        cmd.angular.z = max(-self.MAX_ROT_SPEED, min(self.MAX_ROT_SPEED, cmd.angular.z))
        
        # Only publish if we are not in a waiting state
        if self.state not in [State.STOPPED_FOR_DETECTION, State.DOCKING_AT_P1]:
            self.cmd_pub.publish(cmd)

# --- Main execution ---
def main(args=None):
    '''
    Purpose:
    ---
    Initializes the ROS 2 node and spins it to keep it alive.
    
    Input Arguments:
    ---
    None
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called automatically by the Python interpreter.
    '''
    rclpy.init(args=args)
    
    node = EbotNavTask2A()
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        node.get_logger().info('Keyboard interrupt, stopping node.')
    finally:
        # Clean shutdown
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.try_shutdown()

if __name__ == '__main__':
    main()