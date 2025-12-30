import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Add robot_description to Gazebo resource path
    install_dir = get_package_share_directory('robot_description')
    # Use the parent directory of the share directory to allow finding 'robot_description/assets/...'
    # Actually, get_package_share_directory gives .../share/robot_description
    # We want .../share to be in the path so it can find robot_description inside it.
    
    # However, depending on how colcon installs, sometimes it's cleaner to just add the specific paths.
    # But usually <package>/share is the standard.
    
    library_path = os.path.join(install_dir, '..')
    
    # Check if GZ_SIM_RESOURCE_PATH is already set
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_sim_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + library_path
    else:
        gz_sim_resource_path = library_path

    # Set the environment variable
    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_sim_resource_path
    )

    urdf_file_path = os.path.join(pkg_robot_description, 'robot.urdf')
    gazebo_tags_path = os.path.join(pkg_robot_gazebo, 'config', 'gazebo_tags.xml')

    # Read the URDF file
    with open(urdf_file_path, 'r') as file:
        urdf_content = file.read()

    # Read the Gazebo tags file
    with open(gazebo_tags_path, 'r') as file:
        gazebo_tags_content = file.read()

    # Resolve $(find robot_gazebo) to the actual path
    gazebo_tags_content = gazebo_tags_content.replace('$(find robot_gazebo)', pkg_robot_gazebo)

    # Inject Gazebo tags before the closing </robot> tag
    snippet_start = gazebo_tags_content.find('<robot>') + 7
    snippet_end = gazebo_tags_content.rfind('</robot>')
    inner_gazebo_xml = gazebo_tags_content[snippet_start:snippet_end]
    
    final_urdf = urdf_content.replace('</robot>', inner_gazebo_xml + '\n</robot>')

    # Save the modified URDF to a temporary file
    temp_urdf_path = '/tmp/robot_modified.urdf'
    with open(temp_urdf_path, 'w') as file:
        file.write(final_urdf)
    print("Modified URDF saved to:", temp_urdf_path)

    # Start Gazebo Sim (Ignition)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot State Publisher with the MODIFIED URDF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': final_urdf,
            'use_sim_time': True
        }]
    )

    # Spawn Entity using ros_gz_sim create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', final_urdf,
                   '-name', 'my_robot',
                   '-allow_renaming', 'true',
                   '-z', '0.6'],
        output='screen'
    )

    # Bridge for vital topics
    # Note: gz_ros2_control handles the joint states and commands, so we don't strictly need a bridge for those.
    # But usually we want /clock => /clock etc.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/model/my_robot/joint_state', '/joint_states')
        ],
        output='screen'
    )

    load_robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot_controller', '--controller-manager-timeout', '10'],
        output='screen'
    )

    return LaunchDescription([
        set_env,
        gazebo,
        bridge,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=5.0,
                        actions=[load_robot_controller],
                    )
                ],
            )
        ),
    ])
