#!/bin/bash


#cp -rf /opt/vm2m/lib/ ./vm2m/lib/  
raspi_build=/home/ubuntu/environment/devel/intdash_edge_raspi/intdash-terminal/build-rpi-builder

copy_files(){
    dst=$1
    files=${@:2}

    for file in ${files[@]}; do
        cp ${raspi_build}/${file} ${dst}
    done
}

cd ${raspi_build}
libs=`find -name "*.so*"`
cd -
echo $libs
dst="./vm2m/lib"
copy_files $dst $libs


cd ${raspi_build}
libs=`find -name "intdash-edge-client"`
cd -
dst="./vm2m/sbin"
copy_files $dst $libs

cd ${raspi_build}
libs=`find -name "intdash-edge-logger"`
cd -
dst="./vm2m/sbin"
copy_files $dst $libs

cd ${raspi_build}
libs=`find -name "intdash-edge-manager"`
cd -
dst="./vm2m/sbin"
copy_files $dst $libs


#cp /opt/vm2m/sbin/intdash-edge-client ./vm2m/sbin/
#cp /opt/vm2m/sbin/intdash-edge-logger ./vm2m/sbin/
#cp /opt/vm2m/sbin/intdash-edge-manager ./vm2m/sbin/