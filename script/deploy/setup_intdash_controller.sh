#!/bin/bash

install_target=../../controller_ws/src/intdash_edge_raspi/opt/vm2m

## installer
chmod +x installer.sh
sudo USE_ENV=true ./installer.sh -p $install_target -s dummy2.intdash.jp -u 01234567-0123-0123-0123-0123456789AC -t 0123456789ABCDEFGHIJ0123456789abcdefghij01234568 mjpeg
sudo rm $install_target/var/run

## copy conf file
sudo cp ./conf/controller/* ../../controller_ws/src/intdash_edge_raspi/opt/vm2m/etc

