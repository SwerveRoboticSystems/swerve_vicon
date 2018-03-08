/**
 * @file Sensor.h
 * @breif Header file for Sensor class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-03-06
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <SparkFun_MAG3110.h> // sparkfun magnetometer 3110 library

#define SENSOR_TYPE_CURRENT 1
#define SENSOR_TYPE_DIGITAL 2
#define SENSOR_TYPE_MAG3110 3

#define SENSOR_TYPE_MIN 1
#define SENSOR_TYPE_MAX 3

struct SensorState {
  int type = 0; // type of sensor
  volatile long int timestamp; // time of last update
  volatile double data;
};
struct SensorPins {
  int PIN_DATA;
  int PIN_CLOCK;
  int PIN_INTERRUPT;
};

class Sensor {
public:
  Sensor(int, int, bool, void (*)(void) = NULL);
  /** Sensor(int, int, bool, void (*)(void) = NULL)
   *  @brief Default constructor for non-I2C devices
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  Sensor(int, int, int, bool, void (*)(void) = NULL);
  /** @fn Sensor(int, int, int, bool, void (*)(void) = NULL)
   *  @brief Default constructor for I2C devices
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void setupDataInterrupt(void (*)(void));
  /** @fn void setupDataInterrupt(void (*)(void))
   *  @brief Sets up interrupt for sensor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void setDataScalar(double);
  /** @fn void setDataScalar(double)
   *  @brief Sets a scalar to apply to data from sensor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  double updateData(void);
  /** @fn double updateData(void)
   *  @brief Updates data from sensor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

protected:
  SensorState state;
  SensorPins  pins;

  double DATA_SCALAR = 1.0;
  bool INTERRUPTABLE   = false; // whether data can be updated on interrupt or not
  bool INTERRUPT_SETUP = false; // whether data interrupt has been setup

private:
  MAG3110 _magnetometer;

};

#endif /* SENSOR_H_ */


