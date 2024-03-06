from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ums_fiction_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ums_fiction_driver_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'param', 'mini.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = Node(package='ums_fiction_driver',
                                executable='ums_fiction_driver_node',
                                name=node_name,
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','odom'],
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])