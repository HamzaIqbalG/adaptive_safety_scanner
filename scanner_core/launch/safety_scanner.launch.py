import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. FIND THE LIDAR DRIVER
    # We use the official sllidar_ros2 launch file as a base
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2')
    
    # 2. DEFINE THE NODES
    
    # Node A: The RPLIDAR Driver
    # We include their official launch file but override parameters if needed
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_dir, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser',
            'inverted': 'false',
            'angle_compensate': 'true',
            'serial_baudrate': '460800'
        }.items()
    )

    # Node B: Our Brain (Cluster Detector)
    detector_node = Node(
        package='scanner_core',
        executable='cluster_detector',
        name='cluster_detector',
        output='screen' # Shows print() statements in terminal
    )

    # Node C: RViz2 (Visualization)
    # We point it to the saved config file so it loads your Red Zones automatically
    rviz_config_dir = os.path.join(
        get_package_share_directory('scanner_core'),
        'scanner_core',
        'scanner.rviz' 
    )

    # Note: We need to make sure this path exists after installation. 
    # For now, we will hardcode the path to your source folder to be safe during dev.
    # (Adjust 'hamza' if your username is different)
    rviz_config_path = '/home/hamza/adaptive_safety_scanner_ws/src/adaptive_safety_scanner/scanner_core/scanner_core/scanner.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 3. RETURN THE LAUNCH DESCRIPTION
    return LaunchDescription([
        lidar_launch,
        detector_node,
        rviz_node
    ])