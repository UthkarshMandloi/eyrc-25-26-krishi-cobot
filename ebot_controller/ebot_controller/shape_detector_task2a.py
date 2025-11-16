#!/usr/bin/env python3
'''
# Team ID:          eYRC#4686
# Theme:            Krishi coBot
# Author List:      Uthkarsh Mandloi, Bhavesh kadodiya
# Filename:         shape_detector_task2a.py
# Functions:        __init__, scan_callback, publish_clusters, 
#                   classify_shape_by_line_count, filter_and_convert, 
#                   cluster_points, find_lines_in_cluster_simple, main
# Global variables: None
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, ColorRGBA, Header
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math
import random

class ShapeDetector(Node): 
    """
    This node detects shapes (specifically Squares) from 2D LiDAR data.
    It filters, clusters, and classifies scan points to identify shapes,
    filtering out noise and wall segments based on size and dimensions.
    """
    
    def __init__(self):
        '''
        Purpose:
        ---
        Initializes the ShapeDetector node, setting up all parameters,
        publishers, subscribers, and timers.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        node = ShapeDetector()
        '''
        super().__init__('shape_detector_task2a')

        # --- Filter 1: Cluster Parameters ---
        # CLUSTER_DIST_THRESHOLD: Max distance between points to be in one cluster
        self.CLUSTER_DIST_THRESHOLD = 0.5
        # SCAN_RANGE_MAX: Max distance to consider a LiDAR point (filters distant objects)
        self.SCAN_RANGE_MAX = 2.0 
        
        # --- Filter 2: Size Filter ---
        # SHAPE_MIN_POINTS: A cluster must have at least this many points to be a shape
        self.SHAPE_MIN_POINTS = 10
        # SHAPE_MAX_POINTS: A cluster must have fewer than this many points (filters walls)
        self.SHAPE_MAX_POINTS = 250
        # MAX_SHAPE_DIAMETER: The bounding box of a cluster must be smaller than this (filters long wall segments)
        self.MAX_SHAPE_DIAMETER = 1.0

        # --- Filter 3: RANSAC Parameters ---
        # RANSAC_ITERATIONS: Number of random samples to try when fitting a line
        self.RANSAC_ITERATIONS = 50
        # RANSAC_THRESHOLD: Max distance a point can be from a line to be an "inlier"
        self.RANSAC_THRESHOLD = 0.06
        # MIN_INLIERS_FOR_LINE: A line must have at least this many inlier points
        self.MIN_INLIERS_FOR_LINE = 5  # "Zero Noise" setting

        # --- Cooldown ---
        # DETECTION_COOLDOWN_SEC: Seconds to wait between publishing detections
        self.DETECTION_COOLDOWN_SEC = 15.0 

        # --- Publishers & Subscribers ---
        # shape_pub: Publishes the detected shape status (e.g., "BAD_HEALTH")
        self.shape_pub = self.create_publisher(String, '/found_shape', 10) # ACTIVE
        # scan_sub: Subscribes to the /scan topic for LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data
        )
        # cluster_pub: Publishes debug markers for RViz
        self.cluster_pub = self.create_publisher(MarkerArray, '/shape_clusters', 10)
        
        # last_detection_time: Used to manage the detection cooldown
        self.last_detection_time = self.get_clock().now()
        
        self.get_logger().info('Shape Detector (V12 - Production) Started.')


    def scan_callback(self, msg: LaserScan):
        '''
        Purpose:
        ---
        Main callback function that runs for every LaserScan message.
        It orchestrates the filtering, clustering, and classification pipeline.
        
        Input Arguments:
        ---
        `msg` :  [ sensor_msgs.msg.LaserScan ]
            The incoming LiDAR scan message from the /scan topic.

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically by the scan_sub subscriber.
        '''
        
        # Step 1: Convert LiDAR data to (x, y) points and filter
        points = self.filter_and_convert(msg)
        if len(points) == 0:
            return

        # Step 2: Group nearby points into "blobs" or "clusters"
        clusters = self.cluster_points(points)
        
        # Step 3: Publish all clusters to RViz for debugging
        self.publish_clusters(clusters, msg.header) 
        
        found_shape = None
        for cluster in clusters:
            # --- Filter 1 & 2: Apply Size Filters ---
            # Check point count
            if not (self.SHAPE_MIN_POINTS < len(cluster) < self.SHAPE_MAX_POINTS):
                continue
            
            # Check physical dimensions
            min_x = min(p[0] for p in cluster); max_x = max(p[0] for p in cluster)
            min_y = min(p[1] for p in cluster); max_y = max(p[1] for p in cluster)
            if (max_x - min_x) > self.MAX_SHAPE_DIAMETER or (max_y - min_y) > self.MAX_SHAPE_DIAMETER:
                continue

            # --- If it passes filters, run RANSAC ---
            lines = self.find_lines_in_cluster_simple(cluster, max_lines=5)
            
            # --- Classify based on line count ---
            shape_type = self.classify_shape_by_line_count(lines) 

            if shape_type:
                found_shape = shape_type
                break # Found a shape, stop processing other clusters
        
        # --- Publish if a shape was found and cooldown has passed ---
        if found_shape:
            now = self.get_clock().now()
            if (now - self.last_detection_time).nanoseconds / 1e9 > self.DETECTION_COOLDOWN_SEC:
                
                report_msg = None
                if found_shape == "TRIANGLE":
                    report_msg = "FERTILIZER_REQUIRED"
                elif found_shape == "SQUARE":
                    report_msg = "BAD_HEALTH"
                
                # Only publish if it's a shape we care about (not PENTAGON)
                if report_msg:
                    self.get_logger().info(f"--- DETECTED {report_msg} ---")
                    report = String()
                    report.data = report_msg
                    self.shape_pub.publish(report)
                    # Reset the cooldown timer
                    self.last_detection_time = now


    def publish_clusters(self, clusters, header: Header):
        '''
        Purpose:
        ---
        Publishes all detected clusters to RViz as colored point markers
        for debugging.
        
        Input Arguments:
        ---
        `clusters` :  [ list of np.ndarray ]
            A list where each element is a numpy array of (x, y) points.
        `header` :  [ std_msgs.msg.Header ]
            The original header from the LaserScan message (for frame_id and stamp).

        Returns:
        ---
        None

        Example call:
        ---
        self.publish_clusters(my_clusters, msg.header)
        '''
        marker_array = MarkerArray()
        
        # Create a DELETEALL marker to clear old visualizations
        delete_marker = Marker()
        delete_marker.header.frame_id = header.frame_id
        delete_marker.header.stamp = header.stamp
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = "clusters"
        marker_array.markers.append(delete_marker)

        # Process each cluster
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = header.frame_id
            marker.header.stamp = header.stamp
            marker.ns = "clusters"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05  # Point size
            marker.scale.y = 0.05
            
            # Apply 3-Color Debugging logic
            if len(cluster) < self.SHAPE_MIN_POINTS or len(cluster) > self.SHAPE_MAX_POINTS:
                marker.color.r = 1.0 # RED = Noise or Wall
            else:
                min_x = min(p[0] for p in cluster); max_x = max(p[0] for p in cluster)
                min_y = min(p[1] for p in cluster); max_y = max(p[1] for p in cluster)
                if (max_x - min_x) > self.MAX_SHAPE_DIAMETER or (max_y - min_y) > self.MAX_SHAPE_DIAMETER:
                    marker.color.b = 1.0 # BLUE = Long Wall Segment
                else:
                    marker.color.g = 1.0 # GREEN = Potential Shape!
            marker.color.a = 1.0
            
            # Add all points from the cluster to the marker
            for p in cluster:
                marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
            
            marker_array.markers.append(marker)
            
        # Publish the complete array of markers
        self.cluster_pub.publish(marker_array)

    def classify_shape_by_line_count(self, lines: list):
        '''
        Purpose:
        ---
        Classifies a shape based on the number of lines found by RANSAC.
        This version is for partial marks, detecting squares only.
        
        Input Arguments:
        ---
        `lines` :  [ list ]
            A list of lines found by RANSAC.

        Returns:
        ---
        `shape_name` :  [ str or None ]
            "SQUARE", "PENTAGON", or None.

        Example call:
        ---
        shape = self.classify_shape_by_line_count(my_lines)
        '''
        num_lines = len(lines)
        
        # Based on logs, RANSAC finds 3 or 4 lines on a square
        if num_lines == 3:
            return "SQUARE"
        if num_lines == 4:
            return "SQUARE"
            
        # This will detect the dock but it won't be reported
        if num_lines == 5:
            return "PENTAGON"
            
        # 0, 1, or 2 lines will be ignored (noise or small triangle)
        return None 
        
    def filter_and_convert(self, msg: LaserScan):
        '''
        Purpose:
        ---
        Converts a LaserScan message to a list of (x, y) points,
        filtering out invalid ranges (inf, nan) and points
        beyond SCAN_RANGE_MAX.
        
        Input Arguments:
        ---
        `msg` :  [ sensor_msgs.msg.LaserScan ]
            The incoming LiDAR scan message.

        Returns:
        ---
        `points` :  [ np.ndarray ]
            A numpy array of (x, y) coordinate pairs.

        Example call:
        ---
        points = self.filter_and_convert(msg)
        '''
        points = []
        for i, r in enumerate(msg.ranges):
            # Filter out invalid or distant points
            if math.isinf(r) or math.isnan(r) or r == 0.0 or r > self.SCAN_RANGE_MAX:
                continue
            
            # Convert polar (angle, radius) to cartesian (x, y)
            angle = msg.angle_min + i * msg.angle_increment
            points.append(np.array([r * math.cos(angle), r * math.sin(angle)]))
            
        return np.array(points)

    def cluster_points(self, points: np.ndarray):
        '''
        Purpose:
        ---
        Groups a list of points into clusters based on proximity.
        If the distance between two consecutive points is greater
        than CLUSTER_DIST_THRESHOLD, a new cluster is started.
        
        Input Arguments:
        ---
        `points` :  [ np.ndarray ]
            A numpy array of (x, y) coordinate pairs.

        Returns:
        ---
        `clusters` :  [ list of np.ndarray ]
            A list where each element is a numpy array of (x, y) points.

        Example call:
        ---
        clusters = self.cluster_points(my_points)
        '''
        clusters = []
        if len(points) == 0: return clusters
        
        # Start the first cluster with the first point
        current_cluster = [points[0]]
        for i in range(1, len(points)):
            # Calculate distance between this point and the previous one
            dist = np.linalg.norm(points[i] - points[i-1])
            
            if dist < self.CLUSTER_DIST_THRESHOLD:
                # Point is close, add to current cluster
                current_cluster.append(points[i])
            else:
                # Point is far, save the old cluster and start a new one
                if len(current_cluster) > 1:
                    clusters.append(np.array(current_cluster))
                current_cluster = [points[i]]
        
        # Add the last cluster to the list
        if len(current_cluster) > 1:
            clusters.append(np.array(current_cluster))
            
        return clusters

    def find_lines_in_cluster_simple(self, cluster: np.ndarray, max_lines=5):
        '''
        Purpose:
        ---
        Finds straight lines in a cluster of points using the RANSAC algorithm.
        It iteratively finds the best line, removes its inliers,
        and repeats.
        
        Input Arguments:
        ---
        `cluster` :  [ np.ndarray ]
            An array of (x, y) points for a single object.
        `max_lines` :  [ int ]
            The maximum number of lines to find.

        Returns:
        ---
        `lines` :  [ list ]
            A list of found lines. Each line is a tuple of (p1, p2).

        Example call:
        ---
        lines = self.find_lines_in_cluster_simple(my_cluster, max_lines=5)
        '''
        lines = []
        points = cluster.copy()
        
        for _ in range(max_lines):
            # Stop if not enough points are left to form a line
            if len(points) < self.MIN_INLIERS_FOR_LINE: 
                break 
                
            best_line = None; best_inliers_count = 0; best_inlier_indices = []
            
            # Run RANSAC iterations to find the best possible line
            for _ in range(self.RANSAC_ITERATIONS):
                # 1. Randomly sample 2 points
                idx1, idx2 = random.sample(range(len(points)), 2)
                p1, p2 = points[idx1], points[idx2]
                
                # 2. Define the line equation Ax + By + C = 0
                v = p2 - p1
                if np.linalg.norm(v) == 0: continue
                v_perp = np.array([-v[1], v[0]])
                v_perp_norm = np.linalg.norm(v_perp)
                if v_perp_norm == 0: continue
                
                A, B = v_perp / v_perp_norm
                C = - (A * p1[0] + B * p1[1])
                
                # 3. Find all inliers for this line
                inlier_indices = []
                for i, p in enumerate(points):
                    dist = abs(A * p[0] + B * p[1] + C) 
                    if dist < self.RANSAC_THRESHOLD:
                        inlier_indices.append(i)
                
                # 4. Keep this line if it's the best one so far
                if len(inlier_indices) > best_inliers_count:
                    best_inliers_count = len(inlier_indices)
                    best_line = (p1, p2)
                    best_inlier_indices = inlier_indices
            
            # 5. After iterations, save the best line if it's valid
            if best_inliers_count > self.MIN_INLIERS_FOR_LINE:
                lines.append(best_line)
                # Remove the points from this line so they aren't reused
                points = np.delete(points, best_inlier_indices, axis=0)
            else:
                # No good line found, stop searching
                break 
                
        return lines

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
    
    node = ShapeDetector() 
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        node.get_logger().info('Keyboard interrupt, stopping node.')
    finally:
        # Clean shutdown
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()

# This is the standard entry point for a Python script
if __name__ == '__main__':
    main()