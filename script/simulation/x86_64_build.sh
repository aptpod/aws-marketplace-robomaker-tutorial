#!/bin/bash
workspaces="turtlebot3_teleop_robot_ws turtlebot3_teleop_simulation_ws"

sudo rm /opt/my.xenual.sources.list
sudo cp /usr/local/lib/python3.5/dist-packages/colcon_bundle/installer/assets/xenial.sources.list /opt/my.xenual.sources.list
cat > test.txt <<EOS
# intdash_bridge
deb https://intdash-edge:${BAUTHPASS}@private-repository.aptpod.jp/intdash-edge/linux/ubuntu xenial stable
EOS
sudo sh -c "cat test.txt >> /opt/my.xenual.sources.list"
rm test.txt

for ws in ${workspaces[@]}; do
    echo Build and Bundle for $ws
    cd ../../$ws
    pwd
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --catkin-skip-building-tests
    colcon bundle --apt-sources-list /opt/my.xenual.sources.list
    cd -
done