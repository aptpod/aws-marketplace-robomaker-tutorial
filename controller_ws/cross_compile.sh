#!/bin/bash

# write boto3
cat > /etc/ros/rosdep/sources.list.d/50-boto3.list <<EOS
yaml file:///etc/ros/rosdep/sources.list.d/boto3.yaml
EOS

cat > /etc/ros/rosdep/sources.list.d/boto3.yaml <<EOS
boto3-pip:
  ubuntu:
    pip:
      packages: [boto3]
EOS

cd src/joystick_drivers
git checkout ps3joy/scripts/ps3joy.py
cd -
patch src/joystick_drivers/ps3joy/scripts/ps3joy.py  < ps3joy.py.patch 

ARCH=armhf
DISTRO=raspbian
sudo apt-get update
sudo apt-get install -y apt-transport-https ca-certificates curl gnupg-agent lsb-release
curl -s --compressed ${URL}/intdash-edge/linux/ubuntu/gpg \
  | sudo apt-key add -
echo "deb [arch=${ARCH}] \
        ${URL}/intdash-edge/linux/ubuntu \
        $(lsb_release -cs)  \
        stable" \
       | sudo tee /etc/apt/sources.list.d/intdash-edge.list
sudo apt-get update
sudo apt install intdash-edge -y 
sudo cp -r /opt/vm2m/* /ws/src/intdash_edge_raspi/opt/vm2m/
sudo mkdir -p /ws/src/intdash_edge_raspi/opt/vm2m/var
sudo mkdir -p /ws/src/intdash_edge_raspi/opt/vm2m/etc
sudo cp /conf/manager.conf /ws/src/intdash_edge_raspi/opt/vm2m/etc/

echo "yaml file:///etc/ros/rosdep/sources.list.d/intdash_bridge.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/90-intdash_bridge.list
UBUNTU_CODENAME=`cat /etc/os-release | grep VERSION_CODENAME| cut -f 2 -d '='`
echo -e "intdash_bridge:\n  ubuntu:\n    ${UBUNTU_CODENAME}: [ros-${ROS_DISTRO}-intdash-bridge]\n" | sudo tee /etc/ros/rosdep/sources.list.d/intdash_bridge.yaml
rosdep update

cp apt-sources.yaml /opt/cross/apt-sources.yaml
cp /opt/cross/apt-sources.yaml apt-sources.yaml
echo -e "# intdash_bridge\ndeb ${URL}/intdash-edge/linux/ubuntu ${UBUNTU_CODENAME} stable" >> /opt/cross/apt-sources.yaml

NG_LIST_PATH=`find /usr/local/lib/python* -name apt_package_blacklist.txt`
cp ${NG_LIST_PATH} nglist
echo -e "sudo" >> nglist

apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install --catkin-skip-building-tests
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml --apt-package-blacklist ./nglist
patch -R src/joystick_drivers/ps3joy/scripts/ps3joy.py  < ps3joy.py.patch 
