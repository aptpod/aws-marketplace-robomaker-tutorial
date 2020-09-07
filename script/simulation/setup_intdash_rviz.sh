#!/bin/bash

install_target=../../turtlebot3_teleop_simulation_ws/src/intdash_edge/opt/vm2m

## copy conf file
sudo cp ./conf/rviz/* ../../turtlebot3_teleop_simulation_ws/src/intdash_edge/opt/vm2m/etc

mkdir -p ../../turtlebot3_teleop_robot_ws/src/intdash_edge_raspi/opt/vm2m/bin \
../../turtlebot3_teleop_robot_ws/src/intdash_edge_raspi/opt/vm2m/etc \
../../turtlebot3_teleop_robot_ws/src/intdash_edge_raspi/opt/vm2m/lib \
../../turtlebot3_teleop_robot_ws/src/intdash_edge_raspi/opt/vm2m/sbin \
../../turtlebot3_teleop_robot_ws/src/intdash_edge_raspi/opt/vm2m/share \
../../turtlebot3_teleop_robot_ws/src/intdash_edge_raspi/opt/vm2m/var

mkdir -p ../../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/bin \
../../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/etc \
../../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/lib \
../../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/sbin \
../../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/share \
../../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/var
