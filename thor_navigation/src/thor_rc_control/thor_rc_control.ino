/**
 * @file thor_rc_control.ino
 * @breif Publishes RC controller values to control THOR robot
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-04-27
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include "Logger.h"
#include "RCController6CH.h" // 6 channel RC controller class

// Global Defines
#define PIN_RC_CONTROLLER_CH_1 27
#define INPUT_SCALE 1.5

// Function Prototypes
void controlLoopInterface(void);

// Global Variables
RCController6CH rc_controller(PIN_RC_CONTROLLER_CH_1, &controlLoopInterface);

geometry_msgs::Twist rc_control_msg;

ros::NodeHandle ros_node_handle;
ros::Publisher  rc_control_publisher("/thor/cmd_vel", &rc_control_msg);

void setup() {
  Serial.begin(115200);

  ros_node_handle.initNode();
  ros_node_handle.advertise(rc_control_publisher);
  logger::displayInfo("RC Controller Teensy has been setup");
}

void loop() {
  rc_controller.displayChannels();
  ros_node_handle.spinOnce();
  delay(100);
}

// Function Definitions
void controlLoopInterface(void) {
  
  // Get update controller values
  rc_controller.updateChannels();

  // Construct THOR RC control message and publish to ROS network
  rc_control_msg.linear.x  =  rc_controller.getChannelValue(2) * INPUT_SCALE;
  rc_control_msg.linear.y  = -rc_controller.getChannelValue(1) * INPUT_SCALE;
  rc_control_msg.angular.z = -rc_controller.getChannelValue(4) * INPUT_SCALE;

  rc_control_publisher.publish(&rc_control_msg);
  // logger::displayInfo("published RC controller data to THOR");

}


