#!/bin/bash
sudo apt install intdash-edge -y
sudo cp -r /opt/vm2m/* ../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/
sudo cp -r /opt/vm2m/* ../turtlebot3_teleop_simulation_ws/src/intdash_edge/opt/vm2m/
sudo mkdir -p ../turtlebot3_teleop_robot_ws/src/intdash_edge/opt/vm2m/var
sudo mkdir -p ../turtlebot3_teleop_simulation_ws/src/intdash_edge/opt/vm2m/var
