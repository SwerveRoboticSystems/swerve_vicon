cmake_minimum_required(VERSION 2.8.3)
project(thor_navigation)

find_package(
	catkin REQUIRED COMPONENTS
	tf
	nav_msgs
	geometry_msgs
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

install(
	DIRECTORY config launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


