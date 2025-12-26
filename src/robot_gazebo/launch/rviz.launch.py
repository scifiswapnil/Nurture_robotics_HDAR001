import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    rviz_config_file = os.path.join(pkg_robot_gazebo, 'config', 'view_robot.rviz')

    pkg_robot_description = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_robot_description, 'robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Local Robot State Publisher (for GUI and TFs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }],
            arguments=[urdf_file]
        ),
        # # Slider GUI - republish to /slider_states
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': robot_desc,
        #         'use_sim_time': True
        #     }],
        #     remappings=[
        #         ('/joint_states', '/slider_states')
        #     ]
        # ),
        # Bridge Node - slider_states -> controller commands
        Node(
            package='robot_gazebo',
            executable='slider_bridge.py',
            name='slider_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        ),
    ])
