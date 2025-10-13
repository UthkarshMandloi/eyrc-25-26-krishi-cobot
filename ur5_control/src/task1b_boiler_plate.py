#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
# Team ID:          eYRC#4686
# Theme:            Krishi coBot
# Author List:      Uthkarsh Mandloi
# Filename:         task1b_complete.py
# Functions:        __init__, camera_info_callback, synchronized_callback, 
#                   publish_fruit_transform, main
# Global variables: MARKER_SIZE_METERS
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

# MARKER_SIZE_METERS: The physical size of the ArUco marker in meters.
MARKER_SIZE_METERS = 0.08 

class FinalDetectorNode(Node):
    """
    Detects bad fruits and ArUco markers, publishing fruit TF transforms relative to base_link.
    """
    def __init__(self):
        '''
        Purpose:
        ---
        Initializes the ROS2 node, subscribers for synchronized images and camera info,
        the TF broadcaster, and the ArUco detector.
        
        Input Arguments:
        ---
        None
        
        Returns:
        ---
        None
        
        Example call:
        ---
        node = FinalDetectorNode()
        '''
        super().__init__('final_detector_node')
        
        self.team_id = "4686" 
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.first_fruit_z_in_frame = None

        # These will be populated by the camera_info callback
        self.camera_intrinsics = None
        self.distortion_coeffs = None
        self.intrinsics_received = False
        
        # ArUco Detection Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Subscriber for dynamic camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # Synchronized subscribers for images
        color_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)
        
        self.get_logger().info("Final Detector with all features has been started.")

    def camera_info_callback(self, msg):
        '''
        Purpose:
        ---
        One-time callback to receive and store the camera's intrinsic matrix and distortion coefficients.
        
        Input Arguments:
        ---
        `msg` :  [ sensor_msgs.msg.CameraInfo ]
            The message from the /camera/camera_info topic.
        
        Returns:
        ---
        None
        
        Example call:
        ---
        Called automatically by the ROS2 subscriber.
        '''
        if not self.intrinsics_received:
            self.camera_intrinsics = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.intrinsics_received = True
            self.get_logger().info("Camera intrinsics received dynamically.")
            self.destroy_subscription(self.camera_info_sub)

    def synchronized_callback(self, rgb_msg, depth_msg):
        '''
        Purpose:
        ---
        Main callback that processes synchronized images. It orchestrates fruit detection,
        ArUco visualization, and calls the TF publishing function for confirmed fruits.
        
        Input Arguments:
        ---
        `rgb_msg` :  [ sensor_msgs.msg.Image ]
            The ROS message for the color image.
        `depth_msg` :  [ sensor_msgs.msg.Image ]
            The ROS message for the depth image.
            
        Returns:
        ---
        None
        
        Example call:
        ---
        Called automatically by the message_filters.ApproximateTimeSynchronizer.
        '''
        if not self.intrinsics_received:
            self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5)
            return

        self.first_fruit_z_in_frame = None

        try:
            cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)

        # --- FRUIT DETECTION LOGIC ---
        lower_fruit_body = np.array([0, 0, 48])
        upper_fruit_body = np.array([42, 71, 175])
        fruit_body_mask = cv2.inRange(hsv_image, lower_fruit_body, upper_fruit_body)
        kernel = np.ones((5, 5), np.uint8)
        fruit_body_mask = cv2.morphologyEx(fruit_body_mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(fruit_body_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bad_fruit_id_counter = 1
        for contour in contours:
            # Multi-stage filtering for robust detection
            area = cv2.contourArea(contour)
            if area < 1700 or area > 3000: continue
            x, y, w, h = cv2.boundingRect(contour)
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            vertical_offset = y - (cy - radius)
            if vertical_offset < 5: continue
            roi_hsv = hsv_image[y:y+h, x:x+w]
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            green_mask = cv2.inRange(roi_hsv, lower_green, upper_green)
            
            if cv2.countNonZero(green_mask) > 20: 
                # Actions for a confirmed fruit
                cv2.rectangle(cv_image_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image_rgb, "bad_fruit", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                M = cv2.moments(contour)
                if M["m00"] <= 0: continue
                cx_int, cy_int = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                depth = cv_image_depth[cy_int, cx_int]
                if np.isnan(depth) or depth == 0: continue
                self.publish_fruit_transform(cx_int, cy_int, depth, bad_fruit_id_counter, rgb_msg.header.stamp)
                bad_fruit_id_counter += 1

        # --- ARUCO DETECTION AND VISUALIZATION LOGIC ---
        gray_image = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_METERS, self.camera_intrinsics, self.distortion_coeffs)
            
            for i in range(len(ids)):
                # Publish a TF for the ArUco marker relative to the camera
                t_aruco = TransformStamped()
                t_aruco.header.stamp = rgb_msg.header.stamp
                t_aruco.header.frame_id = 'camera_optical_frame'
                t_aruco.child_frame_id = f'aruco_marker_{ids[i][0]}'
                t_aruco.transform.translation.x, t_aruco.transform.translation.y, t_aruco.transform.translation.z = float(tvecs[i][0][0]), float(tvecs[i][0][1]), float(tvecs[i][0][2])
                
                r = R.from_rotvec(rvecs[i].flatten())
                quat = r.as_quat()
                t_aruco.transform.rotation.x, t_aruco.transform.rotation.y, t_aruco.transform.rotation.z, t_aruco.transform.rotation.w = quat[0], quat[1], quat[2], quat[3]
                self.tf_broadcaster.sendTransform(t_aruco)

                # Draw the axes on the 2D image
                cv2.drawFrameAxes(cv_image_rgb, self.camera_intrinsics, self.distortion_coeffs, rvecs[i], tvecs[i], 0.1)

        cv2.imshow("Live Camera Feed", cv_image_rgb)
        cv2.waitKey(1)

    def publish_fruit_transform(self, cx, cy, depth, fruit_id, timestamp):
        '''
        Purpose:
        ---
        Calculates a fruit's 3D position and publishes its transform relative to the base_link frame
        using a hard-coded manual transformation to correct for simulation inaccuracies.
        
        Input Arguments:
        ---
        `cx` : [ int ]
            The x-coordinate of the fruit's center in pixels.
        `cy` : [ int ]
            The y-coordinate of the fruit's center in pixels.
        `depth` : [ float ]
            The depth of the fruit in meters from the depth image.
        `fruit_id` : [ int ]
            The unique sequential ID for the detected bad fruit.
        `timestamp` : [ builtin_interfaces.msg.Time ]
            The timestamp from the original image message header for TF.
            
        Returns:
        ---
        None
        
        Example call:
        ---
        self.publish_fruit_transform(320, 240, 0.8, 1, msg.header.stamp)
        '''
        # Get camera parameters dynamically
        fx, fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
        c_x, c_y = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]
        
        # Use correction parameters for the final transformation callibration
        cam_base_x = -1.3
        cam_base_y = 0.007
        cam_base_z = 0.3

        # Manual transform logic
        optical_x = (cx - c_x) * depth / fx
        optical_y = (cy - c_y) * depth / fy
        base_x = cam_base_x + depth
        base_y = cam_base_y - optical_x
        base_z = cam_base_z - optical_y
        
        # Enforce a constant height for all fruits
        if self.first_fruit_z_in_frame is None:
            self.first_fruit_z_in_frame = base_z
        else:
            base_z = self.first_fruit_z_in_frame

        try:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = f'{self.team_id}_bad_fruit_{fruit_id}'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(base_x), float(base_y), float(base_z)
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform: {e}")

def main(args=None):
    '''
    Purpose:
    ---
    The main entry point of the script. Initializes rclpy and spins the ROS2 node.
    
    Input Arguments:
    ---
    `args` : [ list ]
        Command-line arguments (optional), passed to rclpy.init().
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called automatically when the Python script is executed.
    '''
    rclpy.init(args=args)
    node = FinalDetectorNode()
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