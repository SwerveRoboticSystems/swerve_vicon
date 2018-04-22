#include <Encoder.h>
#include <Servo.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "Logger.h"

// ---------------------------
// -- RC CONTROLLER DEFINES --
// ---------------------------
#define CH_VALUE_MIN 1200 // min PPM signal in microseconds
#define CH_VALUE_MAX 1800 // max PPM signal in microseconds

#define CH_VALUE_MAP_MIN -100 // minimum value to map the RC controller input to
#define CH_VALUE_MAP_MAX  100 // maximum value to map the RC controller input to

#define CH_MAP_TYPE_BINARY         1 // maps channel values to 0 to 1
#define CH_MAP_TYPE_UNIDIRECTIONAL 2 // maps channel values to 0 to 100
#define CH_MAP_TYPE_BIDIRECTIONAL  3 // maps channel values to -100 to 100

#define MAX_CHANNELS 6 // max channels of the RC controller

#define CH_UPDATE_RATE   100000 // update rate in microseconds
#define PULSE_IN_TIMEOUT 100000 // timeout for pulse function in microseconds

// ------------------------
// -- OMNI ROBOT DEFINES --
// ------------------------
#define SPEED_MIN   0
#define SPEED_MAX 100
#define DEAD_ZONE  10
#define SPIN_GAIN   0.5

#define MOTOR_SPEED_MIN   0
#define MOTOR_SPEED_MAX 100

#define MOTOR_PWM_MIN    0
#define MOTOR_PWM_MAX  255 
#define MOTOR_PPM_MIN 1200
#define MOTOR_PPM_MAX 1800
#define MOTOR_PPM_OFF 1500

// --------------------
// -- SENSOR DEFINES --
// --------------------

#define IMU_UPDATE_RATE 100000 // update rate in microseconds
#define ENCODER_UPDATE_RATE 100000 // update rate in microseconds

// -----------------
// -- LED DEFINES --
// -----------------
#define COLOR_NONE  0
#define COLOR_RED   1
#define COLOR_GREEN 2
#define COLOR_BLUE  3

#define LED_ON  1
#define LED_OFF 0

#define BLINK_OFF     0
#define BLINK_ON      1
#define BLINK_SPEED 250 // speed at which to blink in miliseconds
 
#define SIDE_BOTH     0
#define SIDE_LEFT     1
#define SIDE_RIGHT    2

// --------------------------
// -- FUNCTIONS PROTOTYPES --
// --------------------------
void controlLoop(void);
void readIMU(void);
void sendIMUData(void);
void sendIMUCalibData(void);
void sendEncoderData(void);
void displayIMUState(void);
void displayEncoderState(void);

float mapf(float, float, float, float, float);

void updateControllerChannels(void);
void displayControllerChannels(void);

void runRobotModel(void);
void displayRobotState(void);
void applyDeadZone(int*, int*, int*);

void setLEDState(int, bool blink = false, int side = SIDE_BOTH);
void updateLEDState(void);
void updateLEDBlink(void);
void setLEDColor(bool, bool, bool, int);


// -----------------------
// -- CONTROL VARIABLES --
// -----------------------
IntervalTimer controller_interval_timer;
IntervalTimer imu_interval_timer;
IntervalTimer encoder_interval_timer;

// ---------------------------
// -- STRUCTURE DEFINITIONS --
// ---------------------------
struct RCController6CHState {
  volatile long int timestamp;  // time of last update
  volatile int value_ch_1;
  volatile int value_ch_2;
  volatile int value_ch_3;
  volatile int value_ch_4;
  volatile int value_ch_5;
  volatile int value_ch_6;
  volatile int *values[6] = {&value_ch_1, &value_ch_2, &value_ch_3, 
      &value_ch_4, &value_ch_5, &value_ch_6}; // iterate through channel values
};
struct RCController6CHPins {
  int PIN_CH_1;
  int PIN_CH_2;
  int PIN_CH_3;
  int PIN_CH_4;
  int PIN_CH_5;
  int PIN_CH_6;
  int *PINS[6] = {&PIN_CH_1, &PIN_CH_2, &PIN_CH_3, &PIN_CH_4, 
      &PIN_CH_5, &PIN_CH_6}; // iterate through pins
};

struct IMUState {
  uint8_t calib_sys;
  uint8_t calib_gyro;
  uint8_t calib_accel;
  uint8_t calib_mag;

  imu::Quaternion orientation;
  imu::Vector<3> angular_velocity;
  imu::Vector<3> linear_acceleration;
};

struct OmniRobotState {
  volatile int body_spin;
  volatile double body_speed;
  volatile double body_direction;

  volatile int wheel_speed_left;
  volatile int wheel_speed_right;
  volatile int wheel_speed_tail;
};

struct LEDState {
  int led = COLOR_NONE;
  int red;
  int green;
  int blue;
  int side = SIDE_BOTH;
  bool is_blink = false;
  bool blink;
  uint32_t blink_start;
};

// -------------------------
// -- STRUCTURE VARIABLES --
// -------------------------
RCController6CHState controller_state;
RCController6CHPins controller_pins;

IMUState imu_state;

OmniRobotState omni_robot_state;

LEDState led_state;

// ---------------------
// -- PIN DEFINITIONS --
// ---------------------
const int PIN_CH_1 = 27;

const int PIN_PWM_TAIL  = 4;
const int PIN_PWM_RIGHT = 3;
const int PIN_PWM_LEFT  = 2;
const int PIN_A_TAIL  = 33;
const int PIN_B_TAIL  = 34;
const int PIN_A_RIGHT = 35;
const int PIN_B_RIGHT = 36;
const int PIN_A_LEFT  = 37;
const int PIN_B_LEFT  = 38;

const int PIN_LED_BLUE_LEFT  = 8;
const int PIN_LED_RED_LEFT   = 9;
const int PIN_LED_GREEN_LEFT = 10;
const int PIN_LED_BLUE_RIGHT  = 23;
const int PIN_LED_RED_RIGHT   = 22;
const int PIN_LED_GREEN_RIGHT = 21;

const int PIN_BNO055_SCL = 18;
const int PIN_BNO055_SDA = 19;
const int PIN_BNO055_INT = 20;

const int PIN_ENCODER_A_LEFT  = 0;
const int PIN_ENCODER_B_LEFT  = 1;
const int PIN_ENCODER_A_RIGHT = 11;
const int PIN_ENCODER_B_RIGHT = 12;
const int PIN_ENCODER_A_TAIL  = 25;
const int PIN_ENCODER_B_TAIL  = 26;

// -------------
// -- CLASSES --
// -------------
Encoder encoder_left(PIN_ENCODER_A_LEFT, PIN_ENCODER_B_LEFT);
Encoder encoder_right(PIN_ENCODER_A_RIGHT, PIN_ENCODER_B_RIGHT);
Encoder encoder_tail(PIN_ENCODER_A_TAIL, PIN_ENCODER_B_TAIL);

Adafruit_BNO055 imu_bno055 = Adafruit_BNO055();

// ----------------------
// -- COMMON VARIABLES --
// ----------------------
bool robot_ready = false;

sensor_msgs::Imu imu_msg;
geometry_msgs::Vector3Stamped imu_calib_msg;
geometry_msgs::Vector3Stamped encoder_msg;

ros::NodeHandle node_handle;
ros::Publisher imu_publisher("/thor/imu", &imu_msg);
ros::Publisher imu_calib_publisher("/thor/imu_calib", &imu_calib_msg);
ros::Publisher encoder_publisher("/thor/encoders", &encoder_msg);

// -----------------------
// -- ARDUINO FUNCTIONS --
// -----------------------
void setup() {

  Serial.begin(115200);

  // ----- SETUP CONTROLLER -----
  for (int pin = 0; pin < MAX_CHANNELS; pin++) {
    *controller_pins.PINS[pin] = PIN_CH_1 + pin;
    pinMode(*controller_pins.PINS[pin], INPUT);
  }

  // ----- SETUP ACCELEROMETER -----
  for (int pin = 18; pin < 20; pin++) {
    pinMode(pin, INPUT);
  }

  if(!imu_bno055.begin()) {
    Serial.print("BNO055 was not detected");
    while(1);
  }
  imu_bno055.setExtCrystalUse(true);
  imu_bno055.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);

  // ----- SETUP OMNI ROBOT -----
  for (int pin = 2; pin <= 4; pin++) {
    pinMode(pin, OUTPUT);
  }
  for (int pin = 33; pin <= 38; pin++) {
    pinMode(pin, OUTPUT);
  }

  // ----- SETUP LED -----
  for (int pin = 8; pin <= 10; pin++) {
    pinMode(pin, OUTPUT);
  }
  for (int pin = 21; pin <= 23; pin++) {
    pinMode(pin, OUTPUT);
  }
  setLEDState(COLOR_BLUE);
  led_state.is_blink = false;
  led_state.side     = SIDE_BOTH;
  updateLEDState();

  // ----- SETUP ENCODERS -----
  encoder_left.write(0);
  encoder_right.write(0);
  encoder_tail.write(0);

  // ----- SETUP ROS -----
  node_handle.initNode();
  node_handle.advertise(imu_publisher);
  node_handle.advertise(imu_calib_publisher);
  node_handle.advertise(encoder_publisher);

  // ----- SETUP CONTROLL TIMED INTERRUPT -----
  controller_interval_timer.begin(controlLoop, CH_UPDATE_RATE);

  // ----- SETUP ACCELEROMETER TIMED INTERRUPT -----
  imu_interval_timer.begin(readIMU, IMU_UPDATE_RATE);

  // ----- SETUP ENCODER TIMED INTERRUPT -----
  encoder_interval_timer.begin(sendEncoderData, ENCODER_UPDATE_RATE);

  robot_ready = true;

}

void loop() {
  // displayControllerChannels();
  // displayRobotState();
  displayIMUState();
  // displayEncoderState();
  // Serial.println(digitalRead(PIN_MAG_INT));
  delay(50);
  node_handle.spinOnce();
}

// -------------------------------
// -- TIMED INTERRUPT FUNCTIONS --
// -------------------------------
void controlLoop(void) {
  cli();
  if (robot_ready) {
    updateControllerChannels();
    runRobotModel();
  }
  setLEDState(led_state.led, led_state.is_blink, led_state.side);
  sei();
}

void readIMU(void) {
  cli();
  imu_bno055.getCalibration(&imu_state.calib_sys, &imu_state.calib_gyro, &imu_state.calib_accel, &imu_state.calib_mag);
  imu_state.orientation         = imu_bno055.getQuat();
  imu_state.angular_velocity    = imu_bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_state.linear_acceleration = imu_bno055.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  sendIMUCalibData(); // send imu calibration data to ROS network
  sendIMUData(); // send imu data to ROS network
  sei();
}

void sendIMUData(void) {
  imu_msg.orientation.x = imu_state.orientation.x();
  imu_msg.orientation.y = imu_state.orientation.y();
  imu_msg.orientation.z = imu_state.orientation.z();
  imu_msg.orientation.w = imu_state.orientation.w();
  imu_msg.angular_velocity.x = imu_state.angular_velocity[0];
  imu_msg.angular_velocity.y = imu_state.angular_velocity[1];
  imu_msg.angular_velocity.z = imu_state.angular_velocity[2];
  imu_msg.linear_acceleration.x = imu_state.linear_acceleration[0];
  imu_msg.linear_acceleration.y = imu_state.linear_acceleration[1];
  imu_msg.linear_acceleration.z = imu_state.linear_acceleration[2];

  imu_msg.header.stamp    = node_handle.now();
  imu_msg.header.frame_id = "/imu";
  imu_publisher.publish(&imu_msg);
}

void sendIMUCalibData(void) {
  imu_calib_msg.vector.x = imu_state.calib_sys;
  imu_calib_msg.vector.x = imu_state.calib_gyro;
  imu_calib_msg.vector.x = imu_state.calib_accel;

  imu_calib_msg.header.stamp    = node_handle.now();
  imu_calib_msg.header.frame_id = "/imu";
  imu_calib_publisher.publish(&imu_calib_msg);
}

void sendEncoderData(void) {
  encoder_msg.vector.x = encoder_left.read();
  encoder_msg.vector.y = encoder_right.read();
  encoder_msg.vector.z = encoder_tail.read();

  encoder_msg.header.stamp    = node_handle.now();
  encoder_msg.header.frame_id = "/wheels";
  encoder_publisher.publish(&encoder_msg);
}

void displayIMUState(void) {

  cli();
  String imu_states = "IMU Calibration: ";
  imu_states += String(imu_state.calib_sys) + " (system) | ";
  imu_states += String(imu_state.calib_gyro) + " (gyroscope) | ";
  imu_states += String(imu_state.calib_accel) + " (accelerometer) | ";
  imu_states += String(imu_state.calib_mag) + " (magnetometer)";
  logger::displayInfo(imu_states);
  imu_states = "IMU Orientation: ";
  imu::Vector<3> orientation = imu_state.orientation.toEuler();
  imu_states += String(orientation[0]) + " (x axis) | ";
  imu_states += String(orientation[1]) + " (y axis) | ";
  imu_states += String(orientation[2]) + " (z axis)";
  logger::displayInfo(imu_states);
  imu_states = "IMU Angular Velocity: ";
  imu_states += String(imu_state.angular_velocity[0]) + " (x axis) | ";
  imu_states += String(imu_state.angular_velocity[1]) + " (y axis) | ";
  imu_states += String(imu_state.angular_velocity[2]) + " (z axis)";
  logger::displayInfo(imu_states);
  imu_states = "IMU Linear Acceleration: ";
  imu_states += String(imu_state.linear_acceleration[0]) + " (x axis) | ";
  imu_states += String(imu_state.linear_acceleration[1]) + " (y axis) | ";
  imu_states += String(imu_state.linear_acceleration[2]) + " (z axis)";
  logger::displayInfo(imu_states);
  sei();

}

void displayEncoderState(void) {

  cli();
  String encoder_states = "Encoder States: ";
  encoder_states += String(encoder_left.read()) + " (left) | ";
  encoder_states += String(encoder_right.read()) + " (right) | ";
  encoder_states += String(encoder_tail.read()) + " (tail)";
  logger::displayInfo(encoder_states);
  sei();

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// -----------------------------
// -- RC CONTROLLER FUNCTIONS --
// -----------------------------
void updateControllerChannels(void) {
  int channel_input;

  cli();
  for (int pin = 0; pin < MAX_CHANNELS; pin++) {
    channel_input = pulseIn(*controller_pins.PINS[pin], HIGH, PULSE_IN_TIMEOUT);
    if (channel_input == 0) { // if the controller is off, set input to zero and move to next input
      *controller_state.values[pin] = 0;
      continue;
    }
    // Serial.println(channel_input);

    channel_input = map(channel_input, CH_VALUE_MIN, CH_VALUE_MAX, CH_VALUE_MAP_MIN, CH_VALUE_MAP_MAX);
    channel_input = constrain(channel_input, CH_VALUE_MAP_MIN, CH_VALUE_MAP_MAX);
    *controller_state.values[pin] = channel_input;
  }
  sei();
}

void displayControllerChannels(void) {

  String channel_info = "Channel Values: ";

  cli();
  for (int pin = 0; pin < MAX_CHANNELS; pin++) {
    channel_info += String(*controller_state.values[pin]) + " (CH " + String(pin+1) + ") | ";
  }
  sei();

  channel_info = channel_info.substring(0, channel_info.length()-3);
  logger::displayInfo(channel_info);

}

// --------------------------
// -- OMNI WHEEL FUNCTIONS --
// --------------------------
void runRobotModel(void) {

  // Initialize variables
  int x_input;    // x input in the body frame of the robot
  int y_input;    // y input in the body frame of the robot
  int spin_input; // spin input in the body frame of the robot

  int max_wheel_speed; // max speed of the three wheels

  // Get x, y, and spin input from controller
  cli();
  x_input    =  controller_state.value_ch_2;
  y_input    = -controller_state.value_ch_1;
  spin_input = -controller_state.value_ch_4;
  sei();

  // Apply dead zone for controller
  applyDeadZone(&x_input, &y_input, &spin_input);

  // Calculate speed and direction in the body frame
  omni_robot_state.body_spin      = int(SPIN_GAIN * spin_input);
  omni_robot_state.body_speed     = constrain(sqrt(x_input*x_input + y_input*y_input), SPEED_MIN, SPEED_MAX);
  omni_robot_state.body_direction = atan2(y_input, x_input);

  // Calcualte individual wheel speeds
  omni_robot_state.wheel_speed_left  = int(omni_robot_state.body_speed * cos(-150.0*DEG_TO_RAD + 
      omni_robot_state.body_direction)) + omni_robot_state.body_spin;
  omni_robot_state.wheel_speed_right = int(omni_robot_state.body_speed * cos( -30.0*DEG_TO_RAD + 
      omni_robot_state.body_direction)) + omni_robot_state.body_spin;
  omni_robot_state.wheel_speed_tail  = int(omni_robot_state.body_speed * cos(-270.0*DEG_TO_RAD + 
      omni_robot_state.body_direction)) + omni_robot_state.body_spin;

  // Check if any wheels are maxed out and adjust accordingly to make sure max wheel speed is 100
  max_wheel_speed = max(omni_robot_state.wheel_speed_left, 
      max(omni_robot_state.wheel_speed_right, omni_robot_state.wheel_speed_tail));
  if (max_wheel_speed > SPEED_MAX) {
    double speed_scale = ((SPEED_MAX - omni_robot_state.body_spin) / double(max_wheel_speed));
    omni_robot_state.wheel_speed_left  = floor((omni_robot_state.wheel_speed_left  - omni_robot_state.body_spin) 
        * speed_scale)  + omni_robot_state.body_spin;
    omni_robot_state.wheel_speed_right = floor((omni_robot_state.wheel_speed_right - omni_robot_state.body_spin) 
        * speed_scale)  + omni_robot_state.body_spin;
    omni_robot_state.wheel_speed_left  = floor((omni_robot_state.wheel_speed_tail  - omni_robot_state.body_spin) 
        * speed_scale)  + omni_robot_state.body_spin;
  }

  // Move motors in desired directions
  if (omni_robot_state.wheel_speed_left > 0) {
    digitalWrite(PIN_A_LEFT, HIGH);
    digitalWrite(PIN_B_LEFT, LOW);
  } else if (omni_robot_state.wheel_speed_left < 0) {
    digitalWrite(PIN_A_LEFT, LOW);
    digitalWrite(PIN_B_LEFT, HIGH);
  } else {
    digitalWrite(PIN_A_LEFT, LOW);
    digitalWrite(PIN_B_LEFT, LOW);    
  }
  if (omni_robot_state.wheel_speed_right > 0) {
    digitalWrite(PIN_A_RIGHT, HIGH);
    digitalWrite(PIN_B_RIGHT, LOW);
  } else if (omni_robot_state.wheel_speed_right < 0) {
    digitalWrite(PIN_A_RIGHT, LOW);
    digitalWrite(PIN_B_RIGHT, HIGH);
  } else {
    digitalWrite(PIN_A_RIGHT, LOW);
    digitalWrite(PIN_B_RIGHT, LOW);    
  }
  if (omni_robot_state.wheel_speed_tail > 0) {
    digitalWrite(PIN_A_TAIL, HIGH);
    digitalWrite(PIN_B_TAIL, LOW);
  } else if (omni_robot_state.wheel_speed_tail < 0) {
    digitalWrite(PIN_A_TAIL, LOW);
    digitalWrite(PIN_B_TAIL, HIGH);
  } else {
    digitalWrite(PIN_A_TAIL, LOW);
    digitalWrite(PIN_B_TAIL, LOW);    
  }

  // Move motors at desired speeds
  omni_robot_state.wheel_speed_left = constrain(map(abs(omni_robot_state.wheel_speed_left), MOTOR_SPEED_MIN, 
      MOTOR_SPEED_MAX, MOTOR_PWM_MIN, MOTOR_PWM_MAX), MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  omni_robot_state.wheel_speed_right = constrain(map(abs(omni_robot_state.wheel_speed_right), MOTOR_SPEED_MIN, 
      MOTOR_SPEED_MAX, MOTOR_PWM_MIN, MOTOR_PWM_MAX), MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  omni_robot_state.wheel_speed_tail = constrain(map(abs(omni_robot_state.wheel_speed_tail), MOTOR_SPEED_MIN, 
      MOTOR_SPEED_MAX, MOTOR_PWM_MIN, MOTOR_PWM_MAX), MOTOR_PWM_MIN, MOTOR_PWM_MAX);

  analogWrite(PIN_PWM_LEFT,  omni_robot_state.wheel_speed_left);
  analogWrite(PIN_PWM_RIGHT, omni_robot_state.wheel_speed_right);
  analogWrite(PIN_PWM_TAIL,  omni_robot_state.wheel_speed_tail);

}

void displayRobotState(void) {

  logger::displayInfo("Robot State: " + String(omni_robot_state.wheel_speed_left) + " (left) | " + 
      String(omni_robot_state.wheel_speed_right) + " (right) | " + String(omni_robot_state.wheel_speed_tail) + 
      " (tail) | " + String(omni_robot_state.body_speed) + " (body speed) | " + String(omni_robot_state.body_direction) 
      + " (body direction) | " + String(omni_robot_state.body_spin) + " (body spin)");

}

void applyDeadZone(int *input_1, int *input_2, int *input_3) {

  if (abs(*input_1) < DEAD_ZONE) {
    *input_1 = 0;
  }
  if (abs(*input_2) < DEAD_ZONE) {
    *input_2 = 0;
  }
  if (abs(*input_3) < DEAD_ZONE) {
    *input_3 = 0;
  }

}

// -------------------
// -- LED FUNCTIONS --
// -------------------
void setLEDState(int des_state, bool blink, int side) {

  led_state.is_blink = blink;
  led_state.side     = side;

  if (led_state.led == des_state) {
    if (blink) {
      updateLEDBlink();
    }
  } else {
    led_state.led = des_state;
    updateLEDState();
  }

}

void updateLEDState(void) {

  int red_value   = COLOR_NONE;
  int green_value = COLOR_NONE;
  int blue_value  = COLOR_NONE;

  int des_state = led_state.led;

  if ((led_state.is_blink == true) && (led_state.blink == BLINK_OFF)) {
    des_state = COLOR_NONE;
  }

  switch (des_state) {
    case COLOR_NONE:
      red_value   = LED_OFF;
      green_value = LED_OFF;
      blue_value  = LED_OFF;
      logger::displayDebug("LED color set to: Off");
      break;
    case COLOR_RED:
      red_value   = LED_ON;
      green_value = LED_OFF;
      blue_value  = LED_OFF;
      logger::displayDebug("LED color set to: Yellow");
      break;
    case COLOR_GREEN:
      red_value   = LED_OFF;
      green_value = LED_ON;
      blue_value  = LED_OFF;
      logger::displayDebug("LED color set to: Green");
      break;
    case COLOR_BLUE:
      red_value   = LED_OFF;
      green_value = LED_OFF;
      blue_value  = LED_ON;
      logger::displayDebug("LED color set to: Red");
      break;
  }

  setLEDColor(red_value, green_value, blue_value, led_state.side);

  led_state.red   = red_value;
  led_state.green = green_value;
  led_state.blue  = blue_value;

  led_state.blink_start = millis();

}

void updateLEDBlink(void) {

  if ((millis() - led_state.blink_start) > BLINK_SPEED) {
    led_state.blink = !led_state.blink;
    updateLEDState();
  }

}

void setLEDColor(bool red_value, bool green_value, bool blue_value, int side) {

  cli();
  if (side == SIDE_BOTH) {
    digitalWrite(PIN_LED_RED_LEFT,    red_value);
    digitalWrite(PIN_LED_GREEN_LEFT,  green_value);
    digitalWrite(PIN_LED_BLUE_LEFT,   blue_value);
    digitalWrite(PIN_LED_RED_RIGHT,   red_value);
    digitalWrite(PIN_LED_GREEN_RIGHT, green_value);
    digitalWrite(PIN_LED_BLUE_RIGHT,  blue_value);
    logger::displayDebug("LED displayed on both sides");
  } else if (side == SIDE_LEFT) {
    digitalWrite(PIN_LED_RED_LEFT,    red_value);
    digitalWrite(PIN_LED_GREEN_LEFT,  green_value);
    digitalWrite(PIN_LED_BLUE_LEFT,   blue_value);
    digitalWrite(PIN_LED_RED_RIGHT,   LED_OFF);
    digitalWrite(PIN_LED_GREEN_RIGHT, LED_OFF);
    digitalWrite(PIN_LED_BLUE_RIGHT,  LED_OFF);
    logger::displayDebug("LED displayed on left side");
  } else if (side == SIDE_RIGHT) {
    digitalWrite(PIN_LED_RED_LEFT,    LED_OFF);
    digitalWrite(PIN_LED_GREEN_LEFT,  LED_OFF);
    digitalWrite(PIN_LED_BLUE_LEFT,   LED_OFF);
    digitalWrite(PIN_LED_RED_RIGHT,   red_value);
    digitalWrite(PIN_LED_GREEN_RIGHT, green_value);
    digitalWrite(PIN_LED_BLUE_RIGHT,  blue_value);
    logger::displayDebug("LED displayed on right side");
  }
  sei();

}


