import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_piper_gazebo = FindPackageShare('piper_gazebo')
    pkg_piper_moveit = FindPackageShare('piper_with_gripper_moveit')

    # Launch Gazebo Sim + Robot Spawn + Controllers
    piper_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_piper_gazebo, 'launch', 'piper_with_gripper', 'piper_gazebo.launch.py'])
        ]),
        launch_arguments={'gui': 'true'}.items()
    )

    # Launch MoveIt 2 (Move Group + RViz)
    piper_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_piper_moveit, 'launch', 'piper_moveit.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        piper_gazebo,
        piper_moveit
    ])
