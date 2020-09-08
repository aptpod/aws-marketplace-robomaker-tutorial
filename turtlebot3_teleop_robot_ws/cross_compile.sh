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

ARCH=armhf
DISTRO=raspbian
sudo apt-get update
sudo apt-get install -y apt-transport-https curl gnupg-agent lsb-release
curl -s --compressed \
  -u ${BAUTHUSER}:${BAUTHPASS} \
  "https://private-repository.aptpod.jp/intdash-edge/linux/ubuntu/gpg" \
  | sudo apt-key add -
echo "deb [arch=${ARCH}] \
        https://${BAUTHUSER}:${BAUTHPASS}@private-repository.aptpod.jp/intdash-edge/linux/ubuntu \
        $(lsb_release -cs)  \
        stable" \
       | sudo tee /etc/apt/sources.list.d/intdash-edge.list
sudo apt-get update
sudo apt install intdash-edge -y 
sudo cp -r /opt/vm2m/* /ws/src/intdash_edge_raspi/opt/vm2m/
sudo mkdir -p /ws/src/intdash_edge_raspi/opt/vm2m/var
sudo cp /conf/* /ws/src/intdash_edge_raspi/opt/vm2m/etc

mkdir -p /ws/src/intdash_edge/opt/vm2m/bin \
         /ws/src/intdash_edge/opt/vm2m/etc \
         /ws/src/intdash_edge/opt/vm2m/lib \
         /ws/src/intdash_edge/opt/vm2m/sbin \
         /ws/src/intdash_edge/opt/vm2m/share \
         /ws/src/intdash_edge/opt/vm2m/var

echo "yaml file:///etc/ros/rosdep/sources.list.d/intdash_bridge.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/90-intdash_bridge.list
echo -e "intdash_bridge:\n  ubuntu:\n    xenial: [ros-kinetic-intdash-bridge]\n    bionic: [ros-melodic-intdash-bridge]" | sudo tee /etc/ros/rosdep/sources.list.d/intdash_bridge.yaml
rosdep update

cp apt-sources.yaml /opt/cross/apt-sources.yaml
cp /opt/cross/apt-sources.yaml apt-sources.yaml
echo -e "# intdash_bridge\ndeb https://${BAUTHUSER}:${BAUTHPASS}@private-repository.aptpod.jp/intdash-edge/linux/ubuntu xenial stable" >> /opt/cross/apt-sources.yaml

apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install --catkin-skip-building-tests
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml
