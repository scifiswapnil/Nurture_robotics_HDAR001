#!/usr/bin/env bash

xhost +si:localuser:root

rocker --x11 \
  --nvidia \
  --name nurture_robotics \
  --devices /dev/dri \
  --volume "$HOME:$HOME" \
  --env DISPLAY="$DISPLAY" \
  --network host \
  --privileged \
  ros_jazzy_dev:latest bash /home/robotboy/nuture_robotics/docker_entrypoint.sh

