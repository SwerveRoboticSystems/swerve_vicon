cmake_minimum_required(VERSION 2.8.3)
project(thor_vicon)

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

# Odom Publisher Node
add_executable(vicon_odom_pub src/vicon_odom_pub.cpp)
target_link_libraries(vicon_odom_pub ${catkin_LIBRARIES})
# add_dependencies(vicon_odom_pub [package_name]_generate_messages_cpp)


