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
  -u intdash-edge:${BAUTHPASS} \
  "https://private-repository.aptpod.jp/intdash-edge/linux/ubuntu/gpg" \
  | sudo apt-key add -
echo "deb [arch=${ARCH}] \
        https://intdash-edge:${BAUTHPASS}@private-repository.aptpod.jp/intdash-edge/linux/ubuntu \
        $(lsb_release -cs)  \
        stable" \
       | sudo tee /etc/apt/sources.list.d/intdash-edge.list
sudo apt-get update

echo "yaml file:///etc/ros/rosdep/sources.list.d/intdash_bridge.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/90-intdash_bridge.list
echo -e "intdash_bridge:\n  ubuntu:\n    xenial: [ros-kinetic-intdash-bridge]\n    bionic: [ros-melodic-intdash-bridge]" | sudo tee /etc/ros/rosdep/sources.list.d/intdash_bridge.yaml
rosdep update

cp apt-sources.yaml /opt/cross/apt-sources.yaml
cp /opt/cross/apt-sources.yaml apt-sources.yaml
echo -e "# intdash_bridge\ndeb https://intdash-edge:${BAUTHPASS}@private-repository.aptpod.jp/intdash-edge/linux/ubuntu xenial stable" >> /opt/cross/apt-sources.yaml

apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install --catkin-skip-building-tests
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml
