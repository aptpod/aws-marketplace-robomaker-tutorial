cmake_minimum_required(VERSION 2.8.3)
project(intdash_edge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED rospy)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intdash_edge
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############
#install(DIRECTORY opt
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    USE_SOURCE_PERMISSIONS
#)

install(DIRECTORY opt/vm2m/bin opt/vm2m/etc opt/vm2m/lib opt/vm2m/sbin opt/vm2m/share opt/vm2m/var
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/opt/vm2m
	USE_SOURCE_PERMISSIONS
)

#install(FILES opt/vm2m/etc/manager.conf
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
