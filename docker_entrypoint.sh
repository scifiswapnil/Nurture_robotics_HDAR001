#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
cd /home/robotboy/nuture_robotics || exit 1

# X11
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root

# 🔑 CRITICAL: disable Vulkan/ZINK
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export VK_ICD_FILENAMES=""

# Force Gazebo EGL
export GZ_RENDER_ENGINE=ogre2
export GZ_RENDER_ENGINE_BACKEND=egl

exec bash

