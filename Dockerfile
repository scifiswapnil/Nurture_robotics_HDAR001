FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt install -y \
    ros-jazzy-ros2-control* \
    ros-jazzy-gz* \
    ros-jazzy-moveit-common \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-core \
    ros-jazzy-moveit-core-dbgsym \
    ros-jazzy-moveit-hybrid-planning \
    ros-jazzy-moveit-hybrid-planning-dbgsym \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-kinematics-dbgsym \
    ros-jazzy-moveit-msgs \
    ros-jazzy-moveit-msgs-dbgsym \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-planners-chomp \
    ros-jazzy-moveit-planners-chomp-dbgsym \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-planners-ompl-dbgsym \
    ros-jazzy-moveit-planners-stomp \
    ros-jazzy-moveit-planners-stomp-dbgsym \
    ros-jazzy-moveit-plugins \
    ros-jazzy-moveit-py \
    ros-jazzy-moveit-py-dbgsym \
    ros-jazzy-moveit-resources \
    ros-jazzy-joint-state-publisher* \
    ros-jazzy-om* \
    ros-jazzy-gz-ros2-control \
    net-tools gedit \
 && rm -rf /var/lib/apt/lists/*

