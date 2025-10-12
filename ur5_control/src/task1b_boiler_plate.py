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

# Team ID:          eYRC#4686
# Author List:      Uthkarsh Mandloi
# Filename:         task1b_complete.py
# Functions:        __init__, camera_info_callback, synchronized_callback, 
#                   publish_fruit_transform, main
# Global variables: None


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import tf2_ros
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

class FinalFruitDetectorNode(Node):
    """
    Detects bad fruits using a multi-stage filtering process and publishes their 3D pose as TF transforms.
    """
    def __init__(self):
        '''
        Purpose:
        ---
        Initializes the ROS2 node, synchronized image subscribers, TF2 system, and other necessary components.
        
        Input Arguments:
        ---
        None
        
        Returns:
        ---
        None
        
        Example call:
        ---
        node = FinalFruitDetectorNode()
        '''
        super().__init__('final_fruit_detector_node')
        
        self.team_id = "4686" 

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.intrinsics_received = False
        
        # tf_buffer: Stores incoming transforms for a short time.
        self.tf_buffer = Buffer()
        # tf_listener: Listens for transforms and fills the buffer.
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # tf_broadcaster: Publishes new transforms.
        self.tf_broadcaster = TransformBroadcaster(self)

        # One-time subscriber to get camera calibration data.
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # Synchronized subscribers for color and depth images to ensure they match in time.
        color_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)
        
        self.get_logger().info("Final fruit detector node has been started.")

    def camera_info_callback(self, msg):
        '''
        Purpose:
        ---
        Callback to receive and store the camera's intrinsic parameters, then unsubscribes.
        
        Input Arguments:
        ---
        `msg` :  [ sensor_msgs.msg.CameraInfo ]
            The message containing camera calibration data like focal lengths and principal point.
        
        Returns:
        ---
        None
        
        Example call:
        ---
        Called automatically by the ROS2 subscriber upon receiving a message.
        '''
        if not self.intrinsics_received:
            # Reshape the 1x9 K matrix from the message into a 3x3 numpy array
            self.camera_intrinsics = np.array(msg.k).reshape(3, 3)
            self.intrinsics_received = True
            self.get_logger().info("Camera intrinsics received.")
            # Destroy the subscription after receiving the data once to save resources.
            self.destroy_subscription(self.camera_info_sub)

    def synchronized_callback(self, rgb_msg, depth_msg):
        '''
        Purpose:
        ---
        The main callback that processes synchronized color and depth images to find and publish bad fruits.
        
        Input Arguments:
        ---
        `rgb_msg` :  [ sensor_msgs.msg.Image ]
            The ROS message for the color image.
        `depth_msg` :  [ sensor_msgs.msg.Image ]
            The ROS message for the depth image, synchronized with the color image.
            
        Returns:
        ---
        None
        
        Example call:
        ---
        Called automatically by the message_filters.ApproximateTimeSynchronizer.
        '''
        if not self.intrinsics_received:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        try:
            cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        # Convert the color image to HSV for robust color filtering
        hsv_image = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)

        # --- 1. Primary Detection (Fruit Body Color) ---
        lower_fruit_body = np.array([0, 0, 48])
        upper_fruit_body = np.array([42, 71, 175])
        fruit_body_mask = cv2.inRange(hsv_image, lower_fruit_body, upper_fruit_body)
        
        # Use morphological operations to remove small noise from the mask
        kernel = np.ones((5, 5), np.uint8)
        fruit_body_mask = cv2.morphologyEx(fruit_body_mask, cv2.MORPH_OPEN, kernel)
        
        # Find all potential object contours from the primary mask
        contours, _ = cv2.findContours(fruit_body_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bad_fruit_id_counter = 1
        for contour in contours:
            
            # --- FILTER 1: AREA ---
            # Keep only contours that are within a reasonable size range for a fruit.
            area = cv2.contourArea(contour)
            if area < 1700 or area > 3500:
                continue

            # --- FILTER 2: GEOMETRY ---
            # Keep only contours that match the unique fruit shape (circle extends above bounding box).
            x, y, w, h = cv2.boundingRect(contour)
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            vertical_offset = y - (cy - radius)
            if vertical_offset < 5:
                continue
            
            # --- FILTER 3: GREEN CIRCLE CONFIRMATION ---
            # As a final check, confirm there is a green circle on top of the object.
            roi_hsv = hsv_image[y:y+h, x:x+w]
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            green_mask = cv2.inRange(roi_hsv, lower_green, upper_green)
            
            if cv2.countNonZero(green_mask) > 20: 
                
                # --- ALL CHECKS PASSED: This is a confirmed bad fruit ---
                # Draw visualizations on the output image
                cv2.rectangle(cv_image_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv_image_rgb, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                cv2.putText(cv_image_rgb, "bad_fruit", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Calculate the precise center for depth lookup
                M = cv2.moments(contour)
                if M["m00"] <= 0: continue
                cx_int = int(M["m10"] / M["m00"])
                cy_int = int(M["m01"] / M["m00"])

                # Get the depth value at the center of the fruit
                depth = cv_image_depth[cy_int, cx_int]
                if np.isnan(depth) or depth == 0: continue

                # Call the function to calculate 3D position and publish the TF
                self.publish_fruit_transform(cx_int, cy_int, depth, bad_fruit_id_counter, rgb_msg.header.stamp)
                bad_fruit_id_counter += 1

        cv2.imshow("Live Camera Feed", cv_image_rgb)
        cv2.waitKey(1)

    def publish_fruit_transform(self, cx, cy, depth, fruit_id, timestamp):
        '''
        Purpose:
        ---
        Calculates a fruit's 3D position and publishes it as a TF transform from base_link.
        
        Input Arguments:
        ---
        `cx` : [ int ]
            The x-coordinate of the fruit's center in pixels.
        `cy` : [ int ]
            The y-coordinate of the fruit's center in pixels.
        `depth` : [ float ]
            The depth of the fruit in meters.
        `fruit_id` : [ int ]
            The unique sequential ID for the detected bad fruit.
        `timestamp` : [ builtin_interfaces.msg.Time ]
            The timestamp from the original image message header.
            
        Returns:
        ---
        None
        
        Example call:
        ---
        self.publish_fruit_transform(250, 350, 0.75, 1, msg.header.stamp)
        '''
        # fx, fy are focal lengths; c_x, c_y are principal point coordinates from the intrinsic matrix
        fx, fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
        c_x, c_y = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]
        
        # Convert 2D pixel coordinates to 3D point in the camera's optical frame
        cam_x = (cx - c_x) * depth / fx
        cam_y = (cy - c_y) * depth / fy
        cam_z = depth
        
        try:
            # Look up the transform from the camera frame to the robot's base frame
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_optical_frame', rclpy.time.Time())
            
            point_in_camera = tf2_geometry_msgs.PointStamped()
            point_in_camera.header.frame_id = 'camera_optical_frame'

            # Explicitly convert NumPy floats to standard Python floats
            point_in_camera.point.x = float(cam_x)
            point_in_camera.point.y = float(cam_y)
            point_in_camera.point.z = float(cam_z)

            # NOTE: The extra, incorrect line that was here has been removed.
            
            # Apply the transform to get the point's coordinates in the base_link frame
            point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
            
            # Create and broadcast the final TF message
            t = TransformStamped()
            t.header.stamp, t.header.frame_id = timestamp, 'base_link'
            t.child_frame_id = f'{self.team_id}_bad_fruit_{fruit_id}'
            
            # Convert to float for safety
            t.transform.translation.x = float(point_in_base.point.x)
            t.transform.translation.y = float(point_in_base.point.y)
            t.transform.translation.z = float(point_in_base.point.z)
            t.transform.rotation.w = 1.0 # Identity quaternion for no rotation
            
            self.tf_broadcaster.sendTransform(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main(args=None):
    '''
    Purpose:
    ---
    Initializes the ROS2 system and spins the FinalFruitDetectorNode to begin operation.
    
    Input Arguments:
    ---
    None
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called automatically when the Python script is executed.
    '''
    rclpy.init(args=args)
    node = FinalFruitDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()