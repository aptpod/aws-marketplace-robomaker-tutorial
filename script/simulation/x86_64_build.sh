#!/bin/bash
URL=https://repository.aptpod.jp
# write boto3
echo "yaml file:///etc/ros/rosdep/sources.list.d/boto3.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-boto3.list
echo -e "boto3-pip:\n  ubuntu:\n    pip:\n      packages: [boto3]" | sudo tee /etc/ros/rosdep/sources.list.d/boto3.yaml

sudo apt-get update
rosdep update

workspaces="turtlebot3_teleop_robot_ws turtlebot3_teleop_simulation_ws"

UBUNTU_CODENAME=`cat /etc/os-release | grep VERSION_CODENAME| cut -f 2 -d '='`
SOURCES_LIST_PATH=`find /usr/local/lib/python* -name bionic.sources.list`

sudo rm /opt/my.sources.list
sudo cp ${SOURCES_LIST_PATH} /opt/my.sources.list
echo -e "# intdash_bridge\ndeb ${URL}/intdash-edge/linux/ubuntu ${UBUNTU_CODENAME} stable" >> test.txt

sudo sh -c "cat test.txt >> /opt/my.sources.list"
rm test.txt

for ws in ${workspaces[@]}; do
    echo Build and Bundle for $ws
    cd ../../$ws
    pwd
    sudo rm -rf log/
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --catkin-skip-building-tests
    colcon bundle --apt-sources-list /opt/my.sources.list
    cd -
done