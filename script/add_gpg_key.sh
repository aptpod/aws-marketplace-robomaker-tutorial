#!/bin/bash 
URL=https://repository.aptpod.jp
if [ -z ${DISTRO} ]; then
    echo "Please set DISTRO (e.g. ubuntu, debian, raspbian)";
    exit 1
fi

if [ -z ${ARCH} ]; then
    echo "Please set ARCH (e.g. amd64, arm64, armhf)"
    exit 1
fi

sudo apt-get update
sudo apt-get install -y apt-transport-https curl gnupg-agent lsb-release
curl -s --compressed ${URL}/intdash-edge/linux/${DISTRO}/gpg \
  | sudo apt-key add -
echo "deb [arch=${ARCH}] \
        ${URL}/intdash-edge/linux/${DISTRO} \
        $(lsb_release -cs) \
        stable" \
       | sudo tee /etc/apt/sources.list.d/intdash-edge.list
sudo apt-get update
