#!/usr/bin/env bash

xhost +si:localuser:root

rocker --x11 \
  --nvidia \
  --devices /dev/dri \
  --volume "$HOME:$HOME" \
  --env DISPLAY="$DISPLAY" \
  --network host \
  ros_jazzy_dev:latest bash /home/robotboy/nuture_robotics/docker_entrypoint.sh

