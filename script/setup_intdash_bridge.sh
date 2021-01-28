#!/bin/bash
UBUNTU_CODENAME=`cat /etc/os-release | grep VERSION_CODENAME| cut -f 2 -d '='`

echo "yaml file:///etc/ros/rosdep/sources.list.d/intdash_bridge.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/90-intdash_bridge.list
echo -e "intdash_bridge:\n  ubuntu:\n    ${UBUNTU_CODENAME}: [ros-${ROS_DISTRO}-intdash-bridge]\n" | sudo tee /etc/ros/rosdep/sources.list.d/intdash_bridge.yaml
rosdep update
