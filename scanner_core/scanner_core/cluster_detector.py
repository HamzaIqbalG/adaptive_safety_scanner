import rclpy
from rclpy.node import Node
import numpy as np
from scanner_core.simple_tracker import EuclideanTracker
from scanner_core.safety_zone import SafetyZone
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
        self.epsilon = 0.50 #o.3 original
        self.min_samples = 5 # 5 original
        
        self.get_logger().info('Cluster Detector Node Started')

        self.tracker = EuclideanTracker() #tracker

        # --- SAFETY ZONES ---
        # Define polygon points (x, y) relative to the sensor
        # Warning Zone (Yellow): 1.5m x 1.5m box
        self.warning_zone = SafetyZone("Warning", 901, 
            [(0, 0.75), (1.5, 0.75), (1.5, -0.75), (0, -0.75)], 
            (1.0, 1.0, 0.0, 0.8)) # Yellow

        # Critical Zone (Red): 0.6m x 0.6m box
        self.critical_zone = SafetyZone("Critical", 902, 
            [(0, 0.3), (0.6, 0.3), (0.6, -0.3), (0, -0.3)], 
            (1.0, 0.0, 0.0, 0.8)) # Red

        # Timer to publish zone visuals (1 Hz is enough, they don't move)
        self.create_timer(1.0, self.publish_zones)
    

    def publish_zones(self):
        marker_array = MarkerArray()
        marker_array.markers.append(self.warning_zone.get_marker())
        marker_array.markers.append(self.critical_zone.get_marker())
        self.marker_pub.publish(marker_array)

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

        # --- STEP 3: PREPARE CENTROIDS FOR TRACKER ---
        detected_centroids = []
        for label_id in unique_labels:
            # Get points for this cluster
            cluster_mask = (labels == label_id)
            cluster_points = points_xy[cluster_mask]
            
            # Calculate Centroid
            cx = np.mean(cluster_points[:, 0])
            cy = np.mean(cluster_points[:, 1])
            detected_centroids.append((cx, cy))
        
        # --- STEP 4: UPDATE TRACKER ---
        # Get current ROS time in seconds
        now = self.get_clock().now()
        current_time = now.nanoseconds / 1e9
        
        tracked_objects = self.tracker.update(detected_centroids, current_time)
        
        self.get_logger().info(f'Tracking {len(tracked_objects)} objects', throttle_duration_sec=1.0)

        # --- STEP 5: VISUALIZATION ---
        self.publish_tracked_objects(tracked_objects)

    def publish_tracked_objects(self, tracked_objects):
        marker_array = MarkerArray()

        # --- OPTIONAL: CLEAR OLD MARKERS ---
        # A marker with action=3 (DELETEALL) clears the namespace before adding new ones
        clear_marker = Marker()
        clear_marker.header.frame_id = "laser"
        clear_marker.ns = "boxes"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # We need a separate clear command for each namespace (ids, velocity)
        clear_text = Marker()
        clear_text.header.frame_id = "laser"
        clear_text.ns = "ids"
        clear_text.action = Marker.DELETEALL
        marker_array.markers.append(clear_text)

        clear_arrow = Marker()
        clear_arrow.header.frame_id = "laser"
        clear_arrow.ns = "velocity"
        clear_arrow.action = Marker.DELETEALL
        marker_array.markers.append(clear_arrow)

        for obj in tracked_objects.values():
            # 1. The Box (Green)
            box_marker = Marker()
            box_marker.header.frame_id = "laser"
            box_marker.header.stamp = self.get_clock().now().to_msg()
            box_marker.ns = "boxes"
            box_marker.id = int(obj.id)
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose.position.x = obj.centroid[0]
            box_marker.pose.position.y = obj.centroid[1]
            box_marker.scale.x = 0.2
            box_marker.scale.y = 0.2
            box_marker.scale.z = 0.2
            box_marker.color.a = 0.8
            box_marker.color.g = 1.0
            # --- FIX: Set Lifetime to 0.2 seconds ---
            box_marker.lifetime.sec = 0
            box_marker.lifetime.nanosec = 200000000 
            
            marker_array.markers.append(box_marker)
            
            # 2. The ID Text (White)
            text_marker = Marker()
            text_marker.header.frame_id = "laser"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "ids"
            text_marker.id = int(obj.id) + 1000 
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obj.centroid[0]
            text_marker.pose.position.y = obj.centroid[1]
            text_marker.pose.position.z = 0.3 
            text_marker.scale.z = 0.15 
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            # --- FIX: Set Lifetime ---
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 200000000

            vel_mag = np.linalg.norm(obj.velocity)
            text_marker.text = f"ID:{obj.id}\nV:{vel_mag:.2f}m/s"
            marker_array.markers.append(text_marker)

            # 3. The Velocity Arrow (Red)
            if vel_mag > 0.1: 
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "laser"
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.ns = "velocity"
                arrow_marker.id = int(obj.id) + 2000
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                start = Point()
                start.x = obj.centroid[0]
                start.y = obj.centroid[1]
                
                end = Point()
                end.x = obj.centroid[0] + obj.velocity[0] * 0.5
                end.y = obj.centroid[1] + obj.velocity[1] * 0.5
                
                arrow_marker.points = [start, end]
                
                arrow_marker.scale.x = 0.05 
                arrow_marker.scale.y = 0.1  
                arrow_marker.scale.z = 0.1  
                arrow_marker.color.a = 1.0
                arrow_marker.color.r = 1.0 
                # --- FIX: Set Lifetime ---
                arrow_marker.lifetime.sec = 0
                arrow_marker.lifetime.nanosec = 200000000
                
                marker_array.markers.append(arrow_marker)

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