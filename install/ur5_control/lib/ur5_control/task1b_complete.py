#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
* ===============================================
* Krishi coBot (KC) Theme (eYRC 2025-26)
* ===============================================
*
* This script should be used to implement Task 1B of Krishi coBot (KC) Theme (eYRC 2025-26).
*
* This software is made available on an "AS IS WHERE IS BASIS".
* Licensee/end user indemnifies and will keep e-Yantra indemnified from
* any and all claim(s) that emanate from the use of the Software or
* breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:           eYRC#4686
# Author List:       Uthkarsh Mandloi
# Filename:          task1b_boiler_plate.py
# Functions:
#                    [ Comma separated list of functions in this file ]
# Nodes:             Add your publishing and subscribing node
#                    Publishing Topics  - [ /tf ]
#                    Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]



import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


# runtime parameters
SHOW_IMAGE = True
DISABLE_MULTITHREADING = False

class FruitsTF(Node):
    """
    ROS2 Boilerplate for fruit detection and TF publishing.
    Students should implement detection logic inside the TODO sections.
    """

    def __init__(self):
        super().__init__('fruits_tf')
        self.bridge = CvBridge()
        self.cv_image = None
        self.depth_image = None

        # callback group handling
        if DISABLE_MULTITHREADING:
            self.cb_group = MutuallyExclusiveCallbackGroup()
        else:
            self.cb_group = ReentrantCallbackGroup()

        # Subscriptions - CORRECTED TOPIC NAMES
        self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10, callback_group=self.cb_group)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depthimagecb, 10, callback_group=self.cb_group)

        # Timer for periodic processing
        self.create_timer(0.2, self.process_image, callback_group=self.cb_group)

        if SHOW_IMAGE:
            cv2.namedWindow('fruits_tf_view', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Mask View', cv2.WINDOW_NORMAL) # Add a window for the mask
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.team_id = 4686 # Your Team ID

        self.get_logger().info("FruitsTF boilerplate node started.")

    # ---------------- Callbacks ----------------
    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image.

        Args:
            data (Image): Input depth image frame received from aligned depth camera topic

        Returns:
            None
        '''
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image.

        Args:
            data (Image): Input coloured raw image frame received from image_raw camera topic

        Returns:
            None
        '''
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert color image: {e}")


    def bad_fruit_detection(self, rgb_image):
        '''
        Description:    Function to detect bad fruits in the image frame.
                        Use this function to detect bad fruits and return their center coordinates, distance from camera, angle, width and ids list.

        Args:
            rgb_image (cv2 image): Input coloured raw image frame received from image_raw camera topic

        Returns:
            list: A list of detected bad fruit information, where each entry is a dictionary containing:
                  - 'center': (x, y) coordinates of the fruit center
                  - 'id': unique identifier for the fruit
        '''
        bad_fruits = []
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the greyish-white color in HSV
        # Tuned based on user-provided values from color picker: HSV(30, 10, 53)
        # Converted to OpenCV scale and a range created around it.
        lower_bound = np.array([5, 0, 100])
        upper_bound = np.array([25, 40, 170])

        # Create the mask
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Optional: Apply morphological operations to reduce noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Show the mask in its own window for debugging
        if SHOW_IMAGE:
            cv2.imshow('Mask View', mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        fruit_id_counter = 1
        min_contour_area = 500 # Adjust this value to filter out noise

        for contour in contours:
            if cv2.contourArea(contour) > min_contour_area:
                # Calculate the center of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    continue
                
                # Draw the contour and label on the image for visualization
                cv2.drawContours(rgb_image, [contour], -1, (0, 0, 255), 2)
                cv2.putText(rgb_image, "bad_fruit", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                fruit_info = {
                    'center': (cX, cY),
                    'id': fruit_id_counter
                }
                bad_fruits.append(fruit_info)
                fruit_id_counter += 1

        return bad_fruits


    def process_image(self):
        '''
        Description:    Timer-driven loop for periodic image processing.
        '''
        # Camera intrinsic parameters (approximate for Gazebo)
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640.0
        centerCamY = 360.0
        focalX = 910.0
        focalY = 910.0
        
        if self.cv_image is None or self.depth_image is None:
            return

        visual_image = self.cv_image.copy()
        detected_fruits = self.bad_fruit_detection(visual_image)

        for fruit in detected_fruits:
            cX, cY = fruit['center']
            fruit_id = fruit['id']

            distance_in_mm = self.depth_image[cY, cX]
            if distance_in_mm == 0:
                continue

            distance_from_rgb = float(distance_in_mm) / 1000.0
            
            z = distance_from_rgb
            x = z * (cX - centerCamX) / focalX
            y = z * (cY - centerCamY) / focalY

            cv2.circle(visual_image, (cX, cY), 5, (255, 0, 0), -1)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_link'
            t.child_frame_id = f'{self.team_id}_bad_fruit_{fruit_id}'

            # Transform from camera optical frame (Z-fwd, X-right, Y-down) to ROS standard (X-fwd, Y-left, Z-up)
            t.transform.translation.x = z
            t.transform.translation.y = -x
            t.transform.translation.z = -y
            
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0

            self.tf_broadcaster.sendTransform(t)

        if SHOW_IMAGE:
            cv2.imshow('fruits_tf_view', visual_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FruitsTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down FruitsTF")
        node.destroy_node()
        rclpy.shutdown()
        if SHOW_IMAGE:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

