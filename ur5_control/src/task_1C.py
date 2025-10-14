#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import transforms3d.quaternions as quat
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # This is the publisher that creates the messages
        self.publisher = self.create_publisher(Twist, '/cartesian_servo_node/delta_twist_cmds', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Target Waypoint P1
        self.target_pose = {
            'pos': np.array([-0.214, -0.532, 0.557]),
            'orient': np.array([0.707, 0.707, 0.028, 0.034]) # [w, x, y, z]
        }
        
        self.state = 'INITIALIZING'
        # ... (rest of the __init__ and control_loop code from the final servoing script)