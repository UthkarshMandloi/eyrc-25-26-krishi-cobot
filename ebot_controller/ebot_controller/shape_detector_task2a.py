#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped  # Ensure PointStamped is imported
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import numpy as np
import math
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

class ShapeDetector(Node):
    def _init_(self):
        super()._init_('shape_detector_task2a')
        
        # ROS Communications
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # --- MODIFICATION #1 ---
        # Publish to /found_shape to match your navigation script's subscriber
        self.shape_publisher = self.create_publisher(String, '/found_shape', 10)
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Shape Detection Parameters (TUNABLE) ---
        self.min_cluster_size = 10    
        self.max_cluster_size = 100   
        self.dbscan_eps = 0.15        
        self.ransac_threshold = 0.03  
        self.angle_tolerance = 15.0   
        self.side_len_tolerance = 0.1 
        
        self.get_logger().info('Shape Detector node is READY.')

    def scan_callback(self, msg):
        try:
            points = self.laser_scan_to_points(msg)
            if len(points) == 0:
                return

            clusters = self.cluster_points(points)
            if not clusters:
                return

            try:
                transform = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().warn(f'Could not get transform: {e}')
                return

            for cluster_points in clusters:
                if not (self.min_cluster_size <= len(cluster_points) <= self.max_cluster_size):
                    continue
                
                # Using a placeholder for line fitting, as robust RANSAC is complex
                lines = self.fit_lines_to_cluster(cluster_points)
                
                # --- MODIFICATION #2 ---
                # Classify shape and get the final status (not just the shape name)
                shape_status = self.classify_shape(lines, cluster_points)
                
                if shape_status != "UNKNOWN":
                    # Get the shape's position in the world
                    world_x, world_y = self.get_shape_position(cluster_points, transform)
                    
                    # Publish the full, correct report string
                    report_msg = f"{shape_status},{world_x:.2f},{world_y:.2f}"
                    self.shape_publisher.publish(String(data=report_msg))
                    self.get_logger().info(f'Detected and publishing: {report_msg}')

        except Exception as e:
            self.get_logger().error(f'Scan callback error: {e}')

    def laser_scan_to_points(self, msg):
        points = []
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val) or range_val < msg.range_min or range_val > msg.range_max:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            points.append([x, y])
        return np.array(points)

    def cluster_points(self, points):
        if len(points) < self.min_cluster_size:
            return []
        
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.min_cluster_size).fit(points)
        labels = db.labels_
        unique_labels = set(labels)
        clusters = []
        for k in unique_labels:
            if k == -1: continue
            cluster = points[labels == k]
            clusters.append(cluster)
        return clusters

    def fit_lines_to_cluster(self, cluster_points):
        # This is a placeholder. A robust implementation would use RANSAC.
        return [cluster_points] 

    def classify_shape(self, lines, cluster_points):
        """
        Classifies a shape and returns the required STATUS string.
        """
        if not lines:
            return "UNKNOWN"
            
        min_x, min_y = np.min(cluster_points, axis=0)
        max_x, max_y = np.max(cluster_points, axis=0)
        width = max_x - min_x
        height = max_y - min_y
        
        # Heuristic: Shapes are small (0.1m to 0.5m)
        if (0.1 < width < 0.5) and (0.1 < height < 0.5):
            # Is it squarish?
            if 0.8 < (width / height) < 1.2:
                return "BAD_HEALTH" # Return the status
            else:
                return "FERTILIZER_REQUIRED" # Return the status

        return "UNKNOWN"

    def get_shape_position(self, cluster_points, transform):
        """Finds the centroid and transforms it to the 'odom' frame."""
        centroid = np.mean(cluster_points, axis=0)
        
        point_stamped = PointStamped()
        point_stamped.header.frame_id = 'ebot_base_link' # Assuming scan frame
        point_stamped.point.x = centroid[0]
        point_stamped.point.y = centroid[1]
        point_stamped.point.z = 0.0
        
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        return transformed_point.point.x, transformed_point.point.y

def main(args=None):
    rclpy.init(args=args)
    shape_detector = ShapeDetector()
    rclpy.spin(shape_detector)
    shape_detector.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()