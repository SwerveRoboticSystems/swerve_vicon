/**
 * @file Sensor.cpp
 * @breif Implementation file for Sensor class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-04-25
 */

#include "Logger.h"
#include "Sensor.h"

/* Constructor Functions */
Sensor::Sensor(int sensor_type, int pin_scl, int pin_sda, int pin_int) {

  if (sensor_type == SENSOR_TYPE_IMU) {
    setupSensor(sensor_type, pin_scl, pin_sda, pin_int);
  } else {
    logger::displayError("Invalid sensor type");
  }

}

/* Public Functions */
int Sensor::setupSensor(int sensor_type, int pin_scl, int pin_sda, int pin_int) {

  _pins.PIN_SCL = pin_scl;
  _pins.PIN_SDA = pin_sda;
  _pins.PIN_INT = pin_int;

  pinMode(_pins.PIN_SCL, INPUT);
  pinMode(_pins.PIN_SDA, INPUT);
  pinMode(_pins.PIN_INT, INPUT);

  if (sensor_type == SENSOR_TYPE_IMU) {
    if(!_imu.begin()) {
      logger::displayError("BNO055 IMU was not detected");
      return SENSOR_SETUP_FAILED;
    }
    _imu.setExtCrystalUse(true);
    _imu.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  }

  return SENSOR_SETUP_SUCCESS;

}

IMUState Sensor::readIMUSensor(void) {

  _imu.getCalibration(&_state.imu.calib_sys, &_state.imu.calib_gyro, &_state.imu.calib_accel, &_state.imu.calib_mag);
  _state.imu.orientation         = _imu.getQuat();
  _state.imu.angular_velocity    = _imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  _state.imu.linear_acceleration = _imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  return _state.imu;

}

void Sensor::readIMUSensor(ros::NodeHandle node_handle, sensor_msgs::Imu *imu_state, 
    geometry_msgs::Vector3Stamped *imu_calib, geometry_msgs::Vector3Stamped *heading, String frame_id) {

  imu_state->header.frame_id = frame_id.c_str();
  heading->header.frame_id   = frame_id.c_str();

  _imu.getCalibration(&_state.imu.calib_sys, &_state.imu.calib_gyro, &_state.imu.calib_accel, &_state.imu.calib_mag);
  imu_calib->header.stamp    = node_handle.now();

  _state.imu.orientation         = _imu.getQuat();
  _state.imu.angular_velocity    = _imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  _state.imu.linear_acceleration = _imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_state->header.stamp = node_handle.now();
  heading->header.stamp   = node_handle.now();   

  imu::Vector<3> orientation_euler = _state.imu.orientation.toEuler();

  imu_state->orientation.x = _state.imu.orientation.x();
  imu_state->orientation.y = _state.imu.orientation.y();
  imu_state->orientation.z = _state.imu.orientation.z();
  imu_state->orientation.w = _state.imu.orientation.w();
  imu_state->angular_velocity.x = _state.imu.angular_velocity[0];
  imu_state->angular_velocity.y = _state.imu.angular_velocity[1];
  imu_state->angular_velocity.z = _state.imu.angular_velocity[2];
  imu_state->linear_acceleration.x = _state.imu.linear_acceleration[0] * METERS_TO_G;
  imu_state->linear_acceleration.y = _state.imu.linear_acceleration[1] * METERS_TO_G;
  imu_state->linear_acceleration.z = _state.imu.linear_acceleration[2] * METERS_TO_G;
  imu_calib->vector.x = _state.imu.calib_sys;
  imu_calib->vector.y = _state.imu.calib_gyro;
  imu_calib->vector.z = _state.imu.calib_accel;
  heading->vector.x = orientation_euler[0];
  heading->vector.y = orientation_euler[1];
  heading->vector.z = orientation_euler[2];

}


