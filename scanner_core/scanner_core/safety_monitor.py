import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        # Subscribe to the LIDAR scan topic
        # QoS Profile 10 means "Keep last 10 messages" (Standard reliability)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publisher for status (Safe/Warning/Critical)
        self.publisher_ = self.create_publisher(String, 'safety_status', 10)
        self.get_logger().info('Safety Monitor Node Started')

    def scan_callback(self, msg):
        # msg.ranges is a list of 360+ float values (distances in meters)

        # 1. Filter invalid data 
        # 0.0 often means 'error' or 'out of range' in RPLIDAR driver
        # We also ignore points > 10m to focus on the immediate area
        valid_points = [r for r in msg.ranges if r > 0.05 and r < 10.0]

        if not valid_points:
            return

        # 2. Find the closest point in the scan
        min_distance = min(valid_points)

        # 3. Logic: Determine Safety State
        status_msg = String()

        if min_distance < 0.5:
            # CRITICAL: < 0.5 meters
            self.get_logger().error(f'CRITICAL: Obstacle at {min_distance:.2f}m!')
            status_msg.data = "CRITICAL"
        elif min_distance < 1.0:
            # WARNING: < 1.0 meters
            self.get_logger().warning(f'WARNING: Obstacle at {min_distance:.2f}m')
            status_msg.data = "WARNING"
        else:
            # SAFE
            status_msg.data = "SAFE"

        self.publisher_.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()