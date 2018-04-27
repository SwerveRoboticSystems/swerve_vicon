/** @file odom_pub.cpp
 *  @brief Publishes transform between the world and odometry frame
 *  @author Frederick Wachter - wachterfreddy@gmail.com
 *  @date Created: 2017-11-13
 */

#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define QUEUE_SIZE 100 /**< set queue size for publisher and subscribers */

#define PI 3.14159
#define DEG_TO_RAD (PI/180.0)

double offset_angle = 0.0;
double reference_point_x = 0.0;
double reference_point_z = 0.0;

void robotOdomSubscriber(const geometry_msgs::TransformStamped::ConstPtr&);
/** @fn void robotOdomSubscriber(const geometry_msgs::TransformStamped::ConstPtr&)
 *  @brief Subscriber for odometry of robot
 *  @author Frederick Wachter - wachterfreddy@gmail.com
 *  @date Created: 2017-11-13
 */

void robotOdomRefSubscriber(const geometry_msgs::TransformStamped::ConstPtr&);

#ifndef DOXYGEN_SKIP // don't skip main function
int main(int argc, char** argv){

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle node_handle;

  ros::param::param<double>("/thor/offset_angle", offset_angle, 0.0);

  ROS_INFO("[ODOM] THOR odometry publisher started");
  ros::Subscriber odom_tf_sub = node_handle.subscribe("/vicon/RobotROM01/root", QUEUE_SIZE, &robotOdomSubscriber);
  ros::Subscriber odom_ref_tf_sub = node_handle.subscribe("/vicon/RobotROM01/endBone", QUEUE_SIZE, &robotOdomRefSubscriber);

  ros::spin();
  return 0;

}
#endif /* DOXYGEN_SKIP */

void robotOdomSubscriber(const geometry_msgs::TransformStamped::ConstPtr& odom_msg) {
/** @param odom_msg - incoming odometry message of robot
 */

  double robot_yaw;
  
  static tf::TransformBroadcaster odom_broadcaster;
  tf::Transform transform;

  robot_yaw = atan2(reference_point_x-odom_msg->transform.translation.x, 
      reference_point_z-odom_msg->transform.translation.z);

  transform.setOrigin(tf::Vector3(odom_msg->transform.translation.x, 0, odom_msg->transform.translation.z));
  transform.setRotation(tf::createQuaternionFromRPY(-PI/2, robot_yaw + (offset_angle*DEG_TO_RAD), 0));

  odom_broadcaster.sendTransform(tf::StampedTransform(transform, odom_msg->header.stamp, "world", "thor_odom"));
  
  ROS_INFO("[ODOM] Published odom transform");

}


void robotOdomRefSubscriber(const geometry_msgs::TransformStamped::ConstPtr& odom_msg) {

  reference_point_x = odom_msg->transform.translation.x;
  reference_point_z = odom_msg->transform.translation.z;

}


