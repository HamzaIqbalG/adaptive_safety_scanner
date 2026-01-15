import rclpy
from rclpy.node import Node
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ClusterDetector(Node):
    def __init__(self):
        super().__init__('cluster_detector')
        
        # 1. Subscribe to LIDAR
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # 2. Publisher for Visualizing Clusters in RViz
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # DBSCAN Parameters (The "Tuning Knobs")
        # eps: Max distance between two points to be considered neighbors (0.3m = 30cm)
        # min_samples: Min points to form a cluster (noise reduction)
        self.epsilon = 0.30 
        self.min_samples = 5
        
        self.get_logger().info('Cluster Detector Node Started')

    def scan_callback(self, msg):
        # --- STEP 1: CONVERT POLAR TO CARTESIAN ---
        # Create angles array
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # --- FIX: ROBUST SIZE MATCHING ---
        # Sometimes angles and ranges differ by 1 due to floating point math.
        # We simply truncate both to the shortest length.
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]
        ranges = ranges[:min_len]

        # Filter: Remove 'inf' and '0' and faraway points (> 10m)
        valid_mask = (ranges > 0.05) & (ranges < 10.0) & (np.isfinite(ranges))
        
        if np.sum(valid_mask) < self.min_samples:
            return  # Not enough points to cluster

        # Apply mask
        r_filtered = ranges[valid_mask]
        a_filtered = angles[valid_mask]

        # Polar -> Cartesian Math: X = r*cos(theta), Y = r*sin(theta)
        x = r_filtered * np.cos(a_filtered)
        y = r_filtered * np.sin(a_filtered)
        
        # Stack them into an (N, 2) array for sklearn
        # format: [[x1, y1], [x2, y2], ...]
        points_xy = np.column_stack((x, y))

        # --- STEP 2: CLUSTERING (DBSCAN) ---
        # This is the "AI" part. It groups nearby points.
        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_samples).fit(points_xy)
        labels = clustering.labels_
        
        # unique_labels includes -1 (noise), so we filter that out
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)
            
        self.get_logger().info(f'Found {len(unique_labels)} objects', throttle_duration_sec=1.0)

        # --- STEP 3: VISUALIZATION ---
        self.publish_markers(points_xy, labels, unique_labels)

    def publish_markers(self, points, labels, unique_labels):
        marker_array = MarkerArray()
        
        # For every cluster found...
        for label_id in unique_labels:
            # Get points belonging to this cluster
            cluster_mask = (labels == label_id)
            cluster_points = points[cluster_mask]
            
            # Calculate Centroid (Average X, Average Y)
            centroid_x = np.mean(cluster_points[:, 0])
            centroid_y = np.mean(cluster_points[:, 1])
            
            # Create a Marker (Cube)
            marker = Marker()
            marker.header.frame_id = "laser"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "clusters"
            marker.id = int(label_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position the marker at the centroid
            marker.pose.position.x = centroid_x
            marker.pose.position.y = centroid_y
            marker.pose.position.z = 0.0
            
            # Size (Fixed size 20cm box for now)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Color (Green)
            marker.color.a = 1.0  # Alpha (Transparency)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        # Clear old markers (Optional but good practice to prevent "ghost" boxes)
        # Note: Proper delete logic is complex, for Day 2 we just publish what we see.
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ClusterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()