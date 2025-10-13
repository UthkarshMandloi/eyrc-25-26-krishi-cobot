#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import time
import math

# The order of joints in the 'joint_trajectory_controller'
JOINT_ORDER = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

class JointMoverNode(Node):
    def __init__(self):
        super().__init__('joint_mover_node')

        # Publisher for joint velocity commands
        self.publisher = self.create_publisher(Float64MultiArray, '/delta_joint_cmds', 10)
        
        # Subscriber to get current joint angles
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # TF listener for the initial coordinate check
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State variables
        self.state = 'INITIALIZING'
        self.current_joint_angles = None
        self.joint_names_in_message = None
        self.target_joint_angles = None
        self.joint_to_move_index = 0

        # Controller parameters
        self.joint_gain = 1.0  # Proportional gain for joint movement
        self.angle_tolerance = 0.05  # Radians

        # Main control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Joint Mover node has started.")

    def joint_states_callback(self, msg):
        """Callback to update the current joint angles."""
        if self.joint_names_in_message is None:
            self.joint_names_in_message = list(msg.name)

        # Reorder the received joint angles to match our expected JOINT_ORDER
        ordered_angles = [0.0] * len(JOINT_ORDER)
        for i, name in enumerate(JOINT_ORDER):
            if name in self.joint_names_in_message:
                idx = self.joint_names_in_message.index(name)
                ordered_angles[i] = msg.position[idx]
        
        self.current_joint_angles = np.array(ordered_angles)

    def control_loop(self):
        # Wait until we have received the first joint state message
        if self.current_joint_angles is None:
            self.get_logger().info("Waiting for /joint_states...", throttle_duration_sec=2)
            return

        # --- STATE MACHINE ---
        if self.state == 'INITIALIZING':
            try:
                t = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
                pos = t.transform.translation
                self.get_logger().info(f"Initial Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
                self.get_logger().info("Starting joint movement sequence...")
                self.state = 'PREPARE_NEXT_MOVE'
            except TransformException as ex:
                self.get_logger().warn(f"Waiting for initial transform: {ex}", throttle_duration_sec=1.0)
            return

        elif self.state == 'PREPARE_NEXT_MOVE':
            if self.joint_to_move_index >= len(JOINT_ORDER):
                self.state = 'DONE'
                return
            
            self.get_logger().info(f"--- Preparing to move Joint {self.joint_to_move_index + 1} ({JOINT_ORDER[self.joint_to_move_index]}) ---")
            self.target_joint_angles = np.copy(self.current_joint_angles)
            # Add 90 degrees (pi/2 radians) to the target angle
            self.target_joint_angles[self.joint_to_move_index] += math.pi / 2.0
            self.state = 'MOVING_JOINT'
            time.sleep(1.0) # Pause before starting the move

        elif self.state == 'MOVING_JOINT':
            target_angle = self.target_joint_angles[self.joint_to_move_index]
            current_angle = self.current_joint_angles[self.joint_to_move_index]
            error = target_angle - current_angle

            self.get_logger().info(f"Moving Joint {self.joint_to_move_index + 1}... "
                                 f"Target: {target_angle:.2f}, Current: {current_angle:.2f}, Error: {error:.2f}")

            if abs(error) < self.angle_tolerance:
                self.get_logger().info(f"✅ Joint {self.joint_to_move_index + 1} move complete.")
                self.publisher.publish(Float64MultiArray(data=[0.0]*6)) # Stop motion
                self.joint_to_move_index += 1
                self.state = 'PREPARE_NEXT_MOVE'
                time.sleep(1.0) # Pause before next move
            else:
                velocity = self.joint_gain * error
                # Clamp velocity to a max of 1.0 rad/s for safety
                velocity = np.clip(velocity, -1.0, 1.0)
                
                command = Float64MultiArray()
                command.data = [0.0] * 6
                command.data[self.joint_to_move_index] = velocity
                self.publisher.publish(command)

        elif self.state == 'DONE':
            self.get_logger().info("🎉 Joint movement sequence complete!")
            self.destroy_timer(self.timer)
            time.sleep(1)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = JointMoverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()