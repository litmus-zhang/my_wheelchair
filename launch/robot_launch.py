from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    pkg_share = get_package_share_directory('my_wheelchair')
    
    urdf = os.path.join(pkg_share, 'urdf', 'my_wheelchair.urdf.xacro')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package='my_robot_pkg',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen'),

        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='navigation2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
    ]),
    Node(
            package='voice_control_pkg',
            executable='voice_recognition_node',
            name='voice_recognition_node',
            output='screen'),
        Node(
            package='voice_control_pkg',
            executable='navigation_command_node',
            name='navigation_command_node',
            output='screen'),