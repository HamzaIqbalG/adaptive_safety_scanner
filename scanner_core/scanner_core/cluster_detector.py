import rclpy
from rclpy.node import Node
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# Local Imports
from scanner_core.simple_tracker import EuclideanTracker
from scanner_core.safety_zone import SafetyZone

class ClusterDetector(Node):
    """
    ROS 2 Node for 2D LiDAR Object Detection and Safety Monitoring.
    
    Pipeline:
    1. Receives /scan data (Polar coordinates).
    2. Converts to Cartesian (X, Y) and filters noise/background.
    3. Clusters points using DBSCAN to find objects.
    4. Tracks objects over time to calculate velocity.
    5. Monitors Safety Zones (Warning/Critical) and triggers alerts.
    """
    
    def __init__(self):
        super().__init__('cluster_detector')
        
        # 1. Subscribers & Publishers
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # 2. Perception Tuning (Calibrated for Desktop Environment)
        self.epsilon = 0.30       # Max distance between points in a cluster
        self.min_samples = 10     # High threshold to ignore table surface noise
        self.tracker = EuclideanTracker() 

        # 3. Safety Zones
        # Note: Zones are offset to account for 0.5m blind spot masking
        
        # Warning Zone (Yellow): 1.8m Reach
        self.warning_zone = SafetyZone("Warning", 901, 
            [(0, 0.9), (1.8, 0.9), (1.8, -0.9), (0, -0.9)], 
            (1.0, 1.0, 0.0, 0.8))

        # Critical Zone (Red): 0.9m Reach (Effective 0.5-0.9m)
        self.critical_zone = SafetyZone("Critical", 902, 
            [(0, 0.5), (0.9, 0.5), (0.9, -0.5), (0, -0.5)], 
            (1.0, 0.0, 0.0, 0.8))

        # 4. Timer
        self.get_logger().info('Cluster Detector Node Started - Ready.')
        self.create_timer(1.0, self.publish_zones)

    def publish_zones(self):
        """Publishes the static Safety Zone polygons to RViz."""
        marker_array = MarkerArray()
        marker_array.markers.append(self.warning_zone.get_marker())
        marker_array.markers.append(self.critical_zone.get_marker())
        self.marker_pub.publish(marker_array)

    def check_safety_rules(self, tracked_objects):
        """
        Evaluates current AND future positions of objects against safety zones.
        Returns: 'SAFE', 'WARNING', or 'CRITICAL'
        """
        status = "SAFE"
        lookahead_time = 1.0 # Predict 1 second into the future
        
        for obj in tracked_objects.values():
            x, y = obj.centroid
            vx, vy = obj.velocity
            
            # A. Check CURRENT Position (Immediate Breach)
            if self.critical_zone.contains(x, y):
                status = "CRITICAL"
                break # Stop checking, worst case found
            
            if self.warning_zone.contains(x, y) and status != "CRITICAL":
                status = "WARNING"

            # B. Check FUTURE Position (Time-To-Collision Prediction)
            if status != "CRITICAL":
                pred_x = x + (vx * lookahead_time)
                pred_y = y + (vy * lookahead_time)
                
                if self.critical_zone.contains(pred_x, pred_y):
                    self.get_logger().warn(f"Obj {obj.id} collision predicted in {lookahead_time}s!", throttle_duration_sec=1.0)
                    if status == "SAFE": status = "WARNING"

        # Log Decision
        if status == "CRITICAL":
            self.get_logger().error("!!! CRITICAL BREACH - E-STOP !!!", throttle_duration_sec=0.5)
        elif status == "WARNING":
            self.get_logger().warn("Warning - Object Approaching", throttle_duration_sec=1.0)
            
        return status

    def scan_callback(self, msg):
        # Step 1: Data Conversion
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # Truncate to matching lengths (floating point safety)
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]
        ranges = ranges[:min_len]

        # Step 2: Filtering
        # Mask out near-field noise (<0.5m) and far background (>10m)
        valid_mask = (ranges > 0.50) & (ranges < 10.0) & (np.isfinite(ranges))
        
        if np.sum(valid_mask) < self.min_samples:
            return 

        r_filtered = ranges[valid_mask]
        a_filtered = angles[valid_mask]

        x = r_filtered * np.cos(a_filtered)
        y = r_filtered * np.sin(a_filtered)
        points_xy = np.column_stack((x, y))

        # Step 3: Clustering (DBSCAN)
        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_samples).fit(points_xy)
        labels = clustering.labels_
        unique_labels = set(labels) - {-1} # Exclude noise (-1)

        # Step 4: Centroid Calculation
        detected_centroids = []
        for label_id in unique_labels:
            cluster_mask = (labels == label_id)
            cluster_points = points_xy[cluster_mask]
            cx = np.mean(cluster_points[:, 0])
            cy = np.mean(cluster_points[:, 1])
            detected_centroids.append((cx, cy))
        
        # Step 5: Tracking & Safety Logic
        now = self.get_clock().now()
        current_time = now.nanoseconds / 1e9
        tracked_objects = self.tracker.update(detected_centroids, current_time)
        self.check_safety_rules(tracked_objects)
        self.publish_tracked_objects(tracked_objects)

    def publish_tracked_objects(self, tracked_objects):
        """Visualizes objects (Green Box), IDs (Text), and Velocity (Red Arrow)."""
        marker_array = MarkerArray()

        # Clear old markers to prevent ghosting
        for ns in ["boxes", "ids", "velocity"]:
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            clear_marker.ns = ns
            marker_array.markers.append(clear_marker)

        for obj in tracked_objects.values():
            # 1. Bounding Box
            box = Marker()
            box.header.frame_id = "laser"
            box.header.stamp = self.get_clock().now().to_msg()
            box.ns = "boxes"
            box.id = int(obj.id)
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x, box.pose.position.y = obj.centroid
            box.scale.x = box.scale.y = box.scale.z = 0.2
            box.color.a = 0.8; box.color.g = 1.0
            box.lifetime.nanosec = 200000000 # 0.2s
            marker_array.markers.append(box)
            
            # 2. ID Text
            text = Marker()
            text.header.frame_id = "laser"
            text.ns = "ids"
            text.id = int(obj.id) + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = obj.centroid
            text.pose.position.z = 0.3
            text.scale.z = 0.15
            text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0
            vel_mag = np.linalg.norm(obj.velocity)
            text.text = f"ID:{obj.id}\nV:{vel_mag:.2f}m/s"
            text.lifetime.nanosec = 200000000
            marker_array.markers.append(text)

            # 3. Velocity Arrow
            if vel_mag > 0.1:
                arrow = Marker()
                arrow.header.frame_id = "laser"
                arrow.ns = "velocity"
                arrow.id = int(obj.id) + 2000
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                p1 = Point(); p1.x, p1.y = obj.centroid
                p2 = Point(); p2.x = obj.centroid[0] + obj.velocity[0]*0.5
                p2.y = obj.centroid[1] + obj.velocity[1]*0.5
                arrow.points = [p1, p2]
                arrow.scale.x = 0.05; arrow.scale.y = 0.1; arrow.scale.z = 0.1
                arrow.color.a = 1.0; arrow.color.r = 1.0
                arrow.lifetime.nanosec = 200000000
                marker_array.markers.append(arrow)

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