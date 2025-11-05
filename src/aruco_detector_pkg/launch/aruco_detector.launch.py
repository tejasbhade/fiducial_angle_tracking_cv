from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_detector_pkg')
    config_path = os.path.join(pkg_share, 'config', 'params.yaml')

    declare_display = DeclareLaunchArgument(
        'display_detections',
        default_value='true',
        description='Whether to display detections or not'
    )

    aruco_detection_node = Node(
                                    package='aruco_detector_pkg',
                                    executable='aruco_detector',
                                    name='aruco_detector_node',
                                    output='screen',
                                    parameters=[config_path,
                                                {'display_detections': LaunchConfiguration('display_detections')}],
                                )

    ld = LaunchDescription()
    ld.add_action(declare_display)
    ld.add_action(aruco_detection_node)
    return ld
