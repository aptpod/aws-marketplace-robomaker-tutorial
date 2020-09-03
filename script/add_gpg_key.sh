#!/bin/bash 
test -z ${BAUTHUSER} | echo "Please set BAUTHUSER"
test -z ${BAUTHPASS} | echo "Please set BAUTHPASS"
test -z ${DISTRO} | echo "Please set DISTRO (e.g. ubuntu, debian, raspbian)"
test -z ${ARCH} | echo "Please set ARCH (e.g. amd64, arm64, armhf)"
sudo apt-get update
sudo apt-get install -y apt-transport-https curl gnupg-agent lsb-release
curl -s --compressed \
  -u ${BAUTHUSER}:${BAUTHPASS} \
  "https://private-repository.aptpod.jp/intdash-edge/linux/${DISTRO}/gpg" \
  | sudo apt-key add -
echo "deb [arch=${ARCH}] \
        https://${BAUTHUSER}:${BAUTHPASS}@private-repository.aptpod.jp/intdash-edge/linux/${DISTRO} \
        $(lsb_release -cs) \
        stable" \
       | sudo tee /etc/apt/sources.list.d/intdash-edge.list
sudo apt-get update