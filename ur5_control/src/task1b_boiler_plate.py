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
import math

# Define the physical size of the ArUco marker in meters
MARKER_SIZE_METERS = 0.04 # You may need to adjust this value

class FinalFruitDetectorNode(Node):
    def __init__(self):
        super().__init__('final_fruit_detector_node')
        
        self.team_id = "4686" 

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.distortion_coeffs = None # To store camera distortion
        self.intrinsics_received = False
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ArUco Detection Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        color_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)
        
        self.get_logger().info("Final fruit and ArUco detector node has been started.")

    def camera_info_callback(self, msg):
        if not self.intrinsics_received:
            self.camera_intrinsics = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d) # Save distortion coefficients
            self.intrinsics_received = True
            self.get_logger().info("Camera intrinsics and distortion coeffs received.")
            self.destroy_subscription(self.camera_info_sub)

    def synchronized_callback(self, rgb_msg, depth_msg):
        if not self.intrinsics_received:
            return

        try:
            cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        # =================================================================
        # --- FRUIT DETECTION LOGIC  ---
        # =================================================================
        hsv_image = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)
        lower_fruit_body = np.array([0, 0, 48])
        upper_fruit_body = np.array([42, 71, 175])
        fruit_body_mask = cv2.inRange(hsv_image, lower_fruit_body, upper_fruit_body)
        kernel = np.ones((5, 5), np.uint8)
        fruit_body_mask = cv2.morphologyEx(fruit_body_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(fruit_body_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bad_fruit_id_counter = 1
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1600 or area > 3000:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            vertical_offset = y - (cy - radius)
            if vertical_offset < 5:
                continue
            roi_hsv = hsv_image[y:y+h, x:x+w]
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            green_mask = cv2.inRange(roi_hsv, lower_green, upper_green)
            if cv2.countNonZero(green_mask) > 20: 
                cv2.rectangle(cv_image_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv_image_rgb, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                cv2.putText(cv_image_rgb, "bad_fruit", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                M = cv2.moments(contour)
                if M["m00"] <= 0: continue
                cx_int = int(M["m10"] / M["m00"])
                cy_int = int(M["m01"] / M["m00"])
                depth = cv_image_depth[cy_int, cx_int]
                if np.isnan(depth) or depth == 0: continue
                self.publish_fruit_transform(cx_int, cy_int, depth, bad_fruit_id_counter, rgb_msg.header.stamp)
                bad_fruit_id_counter += 1
        # --- END OF FRUIT DETECTION LOGIC ---


        # =================================================================
        # --- ARUCO MARKER DETECTION LOGIC ---
        # =================================================================
        gray_image = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.aruco_params)
        
        # If any markers are detected
        if ids is not None:
            # Draw borders around detected markers
            cv2.aruco.drawDetectedMarkers(cv_image_rgb, corners, ids)
            
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_METERS, self.camera_intrinsics, self.distortion_coeffs)
            
            # Draw the 3D axes on each marker
            for i in range(len(ids)):
                cv2.drawFrameAxes(cv_image_rgb, self.camera_intrinsics, self.distortion_coeffs, rvecs[i], tvecs[i], 0.3)
        # --- END OF ARUCO DETECTION LOGIC ---

        cv2.imshow("Live Camera Feed", cv_image_rgb)
        cv2.waitKey(1)

    def publish_fruit_transform(self, cx, cy, depth, fruit_id, timestamp):
        # This function remains unchanged and is only for fruits
        fx, fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
        c_x, c_y = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]
        cam_x = (cx - c_x) * depth / fx
        cam_y = (cy - c_y) * depth / fy
        cam_z = depth
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_optical_frame', rclpy.time.Time())
            point_in_camera = tf2_geometry_msgs.PointStamped()
            point_in_camera.header.frame_id = 'camera_optical_frame'
            point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z = float(cam_x), float(cam_y), float(cam_z)
            point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
            t = TransformStamped()
            t.header.stamp, t.header.frame_id = timestamp, 'base_link'
            t.child_frame_id = f'{self.team_id}_bad_fruit_{fruit_id}'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(point_in_base.point.x), float(point_in_base.point.y), float(point_in_base.point.z)
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main(args=None):
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