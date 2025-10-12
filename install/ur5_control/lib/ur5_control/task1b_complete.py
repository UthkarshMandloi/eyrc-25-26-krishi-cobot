#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Import necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image      # ROS message type for images
from cv_bridge import CvBridge         # Package to convert between ROS and OpenCV
import cv2                             # OpenCV library

class ImageSubscriberNode(Node):
    """
    This node subscribes to the raw camera image topic, converts it to an OpenCV image,
    and displays it in a window.
    """

    def __init__(self):
        # Initialize the node with a name
        super().__init__('image_subscriber_node')
        
        # Create a CvBridge object to handle image conversions
        self.bridge = CvBridge()
        
        # Create a subscriber for the raw color image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        self.get_logger().info("Image subscriber node has been started. Waiting for images...")

    def image_callback(self, msg):
        """
        This function is called every time a new image is published on the topic.
        """
        try:
            # Convert the ROS Image message to an OpenCV image ("bgr8" color format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        # Display the image in a window
        cv2.imshow("Live Camera Feed", cv_image)
        
        # This line is crucial for the display window to update
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber_node = ImageSubscriberNode()
    
    try:
        # Keep the node running to receive messages
        rclpy.spin(image_subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly shut down the node and close OpenCV windows
        image_subscriber_node.get_logger().info("Shutting down node.")
        image_subscriber_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()