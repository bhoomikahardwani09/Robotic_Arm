import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('robotic_arm_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robotic_arm.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'config.rviz')

    # Process the Xacro file
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # 1. Publish Robot State (TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # 2. GUI Slider for Joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # 3. RViz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # We will save a config file later, but this line loads it if it exists
            arguments=['-d', rviz_config_file] 
        )
    ])