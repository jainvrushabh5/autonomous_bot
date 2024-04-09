import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('autonomous_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process(xacro_file)
    
    # Hardcoded path to the RViz config file
    rviz_config_path = '/home/rushabh/ros2_ws/src/autonomous_bot/rviz/rviz.rviz'

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Create a joint_state_publisher_gui node
    # node_joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    #     parameters=[params]
    # )

    # Launch RViz2 with the hardcoded config file path
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path]


    return LaunchDescription([
        node_robot_state_publisher,
    ])
