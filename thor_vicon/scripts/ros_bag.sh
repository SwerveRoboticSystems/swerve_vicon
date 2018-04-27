#!/bin/bash
## @file ros_bag.sh
#  @brief Creates a ROS bag of important topics on ROS network for VICON tests with time stamp
#  @author Frederick Wachter - wachterfreddy@gmail.com
#  @date Created: 2018-04-25

BAG_DIR=$1
BAG_NAME=$2

FILE_NAME=$BAG_DIR/bags/$BAG_NAME`date '+%Y-%m-%d_%H-%M-%S.bag'`

echo "[ROS BAG] Recording ROS bag of important topics for VICON testing: $FILE_NAME"
rosbag record -O $FILE_NAME -a # what topics do I want to store?


