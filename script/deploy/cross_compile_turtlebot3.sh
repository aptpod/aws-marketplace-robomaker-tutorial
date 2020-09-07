#!/bin/bash
cd ../../turtlebot3_teleop_robot_ws
sudo docker run -e BAUTHPASS=${BAUTHPASS} -e BAUTHUSER=${BAUTHUSER} -v $(pwd):/ws -v $(pwd)/../script/deploy/conf/turtlebot3:/conf -it ros-cross-compile:armhf /bin/bash -c "cd ws && ./cross_compile.sh"
