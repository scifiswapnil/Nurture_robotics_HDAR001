import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    
    # Initialize Arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo GUI",
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("piper_description"),
                    "urdf",
                    "piper_description_gazebo.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Gazebo Sim Environment
    # Using empty world by default, can be changed
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn Robot
    # Using ros_gz_sim create node to spawn the robot from the topic
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "piper",
            "-topic", "robot_description",
            "-z", "0.1",
        ],
        output="screen",
    )

    # ROS GZ Bridge
    # Bridging clock if necessary
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    gripper8_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper8_controller", "--controller-manager", "/controller_manager"],
    )

    # Gripper mirror node to sync joint8 with joint7
    node_gripper_mirror_controller = Node(
        package='piper_gazebo',
        executable='joint8_ctrl.py',
        output='screen'
    )

    # Ensure controllers start after the spawner
    # In Gazebo Sim/ROS 2 Control, the controller manager is usually spawned by the plugin inside gazebo
    # But we need to make sure the robot is spawned first so the plugin loads? 
    # Actually gz_ros2_control system plugin loads with the URDF. 
    # So once the robot is spawned, the controller manager should be available.

    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
                gripper8_controller_spawner,
            ],
        )
    )

    nodes = [
        gz_sim,
        bridge,
        robot_state_publisher_node,
        spawn_robot,
        delay_controllers_after_spawn,
        node_gripper_mirror_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
