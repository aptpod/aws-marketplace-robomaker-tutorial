#!/bin/bash
echo "yaml file:///etc/ros/rosdep/sources.list.d/intdash_bridge.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/90-intdash_bridge.list
echo -e "intdash_bridge:\n  ubuntu:\n    xenial: [ros-kinetic-intdash-bridge]\n    bionic: [ros-melodic-intdash-bridge]" | sudo tee /etc/ros/rosdep/sources.list.d/intdash_bridge.yaml
rosdep update
