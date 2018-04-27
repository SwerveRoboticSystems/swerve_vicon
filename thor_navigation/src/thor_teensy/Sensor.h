/**
 * @file Sensor.h
 * @breif Header file for Sensor class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-04-25
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

#define SENSOR_SETUP_SUCCESS 0
#define SENSOR_SETUP_FAILED  1

#define SENSOR_TYPE_IMU 1

#define METERS_TO_G (1.0/9.81)

struct IMUState {
  uint8_t calib_sys;
  uint8_t calib_gyro;
  uint8_t calib_accel;
  uint8_t calib_mag;
  imu::Quaternion orientation;
  imu::Vector<3> angular_velocity;
  imu::Vector<3> linear_acceleration;
};

struct SensorState {
  IMUState imu;
};

struct SensorPins {
  int PIN_SCL;
  int PIN_SDA;
  int PIN_INT;
};

class Sensor {
public:

  /* Constructor Functions */
  Sensor(int, int, int, int);
  /** @fn Sensor(int, int, int, int)
   *  @brief Default constructor for an IMU with an interrupt pin
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

  /* Public Functions */
  int setupSensor(int, int, int, int);
  /** @fn void setupMotor(int, int, int)
   *  @brief Sets up the pins of the sensor class
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

  IMUState readIMUSensor(void);
  /** @fn IMUState readIMUSensor(void)
   *  @brief Reads IMU sensor data
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

  void readIMUSensor(ros::NodeHandle, sensor_msgs::Imu*, geometry_msgs::Vector3Stamped*, 
      geometry_msgs::Vector3Stamped*, String);
  /** @fn void readIMUSensor(ros::NodeHandle, sensor_msgs::Imu*, geometry_msgs::Vector3Stamped*, 
          geometry_msgs::Vector3Stamped*, String)
   *  @brief Reads IMU sensor data and timestamps data using ROS node handle and adds specificed frame ID
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

private:
  
  /* Private Variables */
  SensorState _state;
  SensorPins  _pins;

  Adafruit_BNO055 _imu = Adafruit_BNO055();

};

#endif /* SENSOR_H_ */


