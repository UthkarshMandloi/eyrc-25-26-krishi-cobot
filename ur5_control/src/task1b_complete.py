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

        # Subscriptions
        self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10, callback_group=self.cb_group)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depthimagecb, 10, callback_group=self.cb_group)

        # Timer for periodic processing
        self.create_timer(0.2, self.process_image, callback_group=self.cb_group)

        if SHOW_IMAGE:
            cv2.namedWindow('fruits_tf_view', cv2.WINDOW_NORMAL)
        
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

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #  -> Use `data` variable to convert ROS Image message to CV2 Image type
        #  -> HINT: You may use CvBridge to do the same
        #  -> Store the converted image into `self.depth_image`

        ############################################
        try:
            # The depth image from simulation is typically encoded in 16-bit unsigned int (millimeters)
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

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :
        #  -> Use `data` variable to convert ROS Image message to CV2 Image type
        #  -> HINT: You may use CvBridge to do the same
        #  -> Store the converted image into `self.cv_image`
        #  -> Check if you need any rotation or flipping of the image 
        #     (as input data may be oriented differently than expected).
        #     You may use cv2 functions such as `cv2.flip` or `cv2.rotate`.

        ############################################
        try:
            # Convert the ROS Image message to a BGR OpenCV image.
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
                  - 'distance': distance from the camera in meters
                  - 'angle': angle of the fruit in degrees
                  - 'width': width of the fruit in pixels
                  - 'id': unique identifier for the fruit
        '''
        ############ ADD YOUR CODE HERE ############
        # INSTRUCTIONS & HELP :
        #  ->  Implement bad fruit detection logic using image processing techniques
        #  ->  You may use techniques such as color filtering, contour detection, etc.
        #  ->  For each detected bad fruit, create a dictionary with its information and append
        #      to the bad_fruits list
        #  ->  Return the bad_fruits list at the end of the function
        # Step 1: Convert RGB image to HSV color space
        #  - Use cv2.cvtColor to convert the input image to HSV for better color segmentation

        # Step 2: Define lower and upper HSV bounds for "bad fruit" color
        #  - Choose HSV ranges that correspond to the color of bad fruits (e.g., brown/black spots)

        # Step 3: Create a binary mask using cv2.inRange
        #  - This mask highlights pixels within the specified HSV range

        # Step 4: Find contours in the mask
        #  - Use cv2.findContours to detect continuous regions (potential bad fruits)

        # Step 5: Loop through each contour
        #  - Filter out small contours by area threshold to remove noise
        #  - For each valid contour:
        #      a. Compute bounding rectangle (cv2.boundingRect)
        #      b. Calculate center coordinates (cX, cY)
        #      c. (Optional) Calculate distance and angle if depth data is available
        #      d. Store fruit info (center, distance, angle, width, id) in a dictionary
        #      e. Append dictionary to bad_fruits list

        # Step 6: Return the bad_fruits list
        bad_fruits = []

        # TODO: Implement bad fruit detection logic here
        # You may use image processing techniques such as color filtering, contour detection, etc.
        # For each detected bad fruit, append its information to the bad_fruits list
        
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the greyish-white color in HSV
        # This range might need tuning depending on the simulation's lighting
        lower_bound = np.array([0, 0, 160])
        upper_bound = np.array([180, 50, 255])

        # Create the mask
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Optional: Apply morphological operations to reduce noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

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

        Returns:
            None
        '''
        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640.0 # From default Gazebo camera parameters
        centerCamY = 360.0 # From default Gazebo camera parameters
        focalX = 910.0 # Approximate value for Gazebo camera
        focalY = 910.0 # Approximate value for Gazebo camera
                

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #  ->  Get fruit center, distance from rgb, angle, width and ids list from 'detect_fruit_center' defined above

        #  ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #  
        #  ->  Use center_fruit_list to get realsense depth and log them down.

        #  ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #      x = distance_from_rgb * (cX - centerCamX) / focalX
        #      y = distance_from_rgb * (cY - centerCamY) / focalY
        #      z = distance_from_rgb
        #      where, 
        #            cX, and cY from 'center_fruit_list'
        #            distance_from_rgb is depth of object calculated in previous step
        #            sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #  ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.circle' function 

        #  ->  Here, till now you receive coordinates from camera_link to fruit center position. 
        #      So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #      so that we will collect its position w.r.t base_link in next step.
        #      Use the following frame_id-
        #            frame_id = 'camera_link'
        #            child_frame_id = 'cam_<fruit_id>'        Ex: cam_20, where 20 is fruit ID

        #  ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #      You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #  ->  And now publish TF between object frame and base_link
        #      Use the following frame_id-
        #            frame_id = 'base_link'
        #            child_frame_id = f'{teamid}_bad_fruit_{fruit_id}'   Ex: 5_bad_fruit_1, where 5 is team ID and 1 is fruit ID

        #  ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #      Refer MD book on portal for sample image -> https://portal.e-yantra.org/
        
        if self.cv_image is None or self.depth_image is None:
            return

        # Create a copy for visualization to avoid modifying the original image
        visual_image = self.cv_image.copy()
        
        detected_fruits = self.bad_fruit_detection(visual_image)

        for fruit in detected_fruits:
            cX, cY = fruit['center']
            fruit_id = fruit['id']

            # Get depth value from depth image (in millimeters)
            distance_in_mm = self.depth_image[cY, cX]

            # If depth is 0, it means the sensor couldn't get a reading, so skip this detection
            if distance_in_mm == 0:
                continue

            # Convert depth from millimeters to meters
            distance_from_rgb = float(distance_in_mm) / 1000.0
            
            # The Gazebo camera link has Y pointing down, Z pointing forward, X pointing right.
            # We need to adjust the formula to match ROS conventions (X forward, Y left, Z up).
            # The transform from the optical frame to the camera_link frame is usually handled by the static_transform_publisher in the launch file.
            # Let's calculate in the camera optical frame first (Z forward, X right, Y down).
            z = distance_from_rgb
            x = z * (cX - centerCamX) / focalX
            y = z * (cY - centerCamY) / focalY

            # Mark the center point on the visual image
            cv2.circle(visual_image, (cX, cY), 5, (255, 0, 0), -1)

            # Publish the transform from 'camera_link' to the fruit's frame
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_link'
            t.child_frame_id = f'{self.team_id}_bad_fruit_{fruit_id}'

            # The calculated coordinates are in the camera's optical frame.
            # We publish them directly. The static transform between `camera_optical_frame`
            # and `camera_link` should handle the coordinate system change. For Gazebo, this is often the same.
            t.transform.translation.x = z
            t.transform.translation.y = -x
            t.transform.translation.z = -y
            
            # No rotation, so use a "zero" quaternion (w=1)
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
