/** 
 * @file thor.ino
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2015-04-28
 */

//----- INCLUDED LIBRARIES -----\\
#include <PID_v1.h>
#include <Math.h>
//----- INCLUDED LIBRARIES -----\\

//----- MOTOR PINS -----\\
const int Motor_1_EncoderInterrupt = 2;
const int Motor_2_EncoderInterrupt = 3;
const int Motor_3_EncoderInterrupt = 4;

// Right Motor
const int Motor_1_PWM = 5; // D2
const int Motor_1_A = 40; // IN1
const int Motor_1_B = 38; // IN2
const int Motor_1_Enable = 44; // ENABLE
const int Motor_1_D1 = 42; // D1

// Back Motor
const int Motor_2_PWM = 6;
const int Motor_2_A = 32;
const int Motor_2_B = 30;
const int Motor_2_Enable = 36;
const int Motor_2_D1 =34;

// Left Motor
const int Motor_3_PWM = 7;
const int Motor_3_A = 24;
const int Motor_3_B = 22;
const int Motor_3_Enable = 28;
const int Motor_3_D1 = 26;
//----- MOTOR PINS -----\\

//----- MOTOR VARIABLES -----\\
double Motor_1_MotorRotations_CurrentSample = 0;
double Motor_2_MotorRotations_CurrentSample = 0;
double Motor_3_MotorRotations_CurrentSample = 0;

double Motor_1_MotorRotations_LastSample = 0;
double Motor_2_MotorRotations_LastSample = 0;
double Motor_3_MotorRotations_LastSample = 0;

volatile long Motor_1_Ticks = 0;
volatile long Motor_2_Ticks = 0;
volatile long Motor_3_Ticks = 0;

int movingAverageIndex = 0;
int movingAverageSize = 5;
double Motor_1_MovingAverageMatrix[5];
double Motor_2_MovingAverageMatrix[5];
double Motor_3_MovingAverageMatrix[5];
double Motor_1_MovingAverageValue = 0;
double Motor_2_MovingAverageValue = 0;
double Motor_3_MovingAverageValue = 0;

long Motor_1_Rotations = 0;
long Motor_2_Rotations = 0;
long Motor_3_Rotations = 0;

int Motor_1_Direction = 1;
int Motor_2_Direction = 1;
int Motor_3_Direction = 1;

int Motor_Left_Direction = 0;
int Motor_Right_Direction = 0;
int Motor_Back_Direction = 0;

double robo_speed = 0;
double theta = 0*(3.14159/180);
float spin_gain = 0.25; //Arbitrary gain to adjust spin speed
float speed_gain = 0.40; //Arbitrary gain to adjust top speed
//----- MOTOR VARIABLES -----\\

//----- RPM TO PWM MATHMATICAL MODEL PARAMETERS -----\\
double equation_a = 0.00067;
double equation_b = 1.93701;
double equation_c = 36.14050;
double equation_d = 0.24691;
double equation_e = 1.0;
int equation_LowerLimit = 20;
//----- RPM TO PWM MATHMATICAL MODEL PARAMETERS -----\\

//----- CONTROLLER -----\\
const int CH6 = 13; // Left Knob
const int CH5 = 12; // Right Knob
const int CH4 = 11; // Left-Right on Left Stick
const int CH3 = 10; // Up-Down on Right Stick
const int CH2 = 9;  // Up-Down on Left Stick
const int CH1 = 8;  // Left-Right on Right Stick

const double forwardAdjust = -1550.0;
const double reverseAdjust = -1450.0;
const double PWMAdjustment = 2.0/3.0;

int CH2_Input = 0;
int CH4_Input = 0;
int CH3_Input = 0;
int CH1_Input = 0;

double CH2_Offset = 0;
double CH4_Offset = 0;
double CH3_Offset = 0;
double CH1_Offset = 0;

double straight = 0;
double spin = 0;
double strafe = 0;
double CH3_PWMVal = 0;

int medianValue = 1500;
//----- CONTROLLER -----\\

//----- PROGRAM VARIABLES -----\\
double currentTime = 0;
double previousTime = 0;
int calculatePIDInterval = 20;

double ticksPerRevolution = 480.0;
double milliPerMin = 1000;
int motorsRunning = 3;
int deadZone = 20;

int displayIndex = 0;
int displayIterations = 15;
//----- PROGRAM VARIABLES -----\\

//----- PID -----\\
double PIDAdjust = 1.0;
double Setpoint[3], Input[3], Output[3];
double kp = 2;
double ki = 1;
double kd = 1;
//PID Motor_1_PID(&Input[0], &Output[0], &Setpoint[0], kp, ki, kd, DIRECT);
//PID Motor_2_PID(&Input[1], &Output[1], &Setpoint[1], kp, ki, kd, DIRECT);
//PID Motor_3_PID(&Input[2], &Output[2], &Setpoint[2], kp, ki, kd, DIRECT);
//----- PID -----\\

//void setup(void);
//void loop(void);
//void Motor_1_InterruptFunction(void);
//void Motor_2_InterruptFunction(void);
//void Motor_3_InterruptFunction(void);
//void readController(void);
//void Motor_1_MovingAverageFunction(void);
//void Motor_2_MovingAverageFunction(void);
//void Motor_3_MovingAverageFunction(void);

void setup() {
  Serial.begin(9600);
  
  //----- ENCODERS -----\\
  attachInterrupt(Motor_1_EncoderInterrupt, Motor_1_InterruptFunction, RISING);
  attachInterrupt(Motor_2_EncoderInterrupt, Motor_2_InterruptFunction, RISING);
  attachInterrupt(Motor_3_EncoderInterrupt, Motor_3_InterruptFunction, RISING);
  //----- ENCODERS -----\\

  //----- PID -----//
  Motor_1_PID.SetOutputLimits(0,255);
  Motor_1_PID.SetMode(AUTOMATIC);
  Motor_2_PID.SetOutputLimits(0,255);
  Motor_2_PID.SetMode(AUTOMATIC);
  Motor_3_PID.SetOutputLimits(0,255);
  Motor_3_PID.SetMode(AUTOMATIC);
  //----- PID -----//
  
  //----- MOTORS -----//
  pinMode(Motor_1_PWM,OUTPUT);
  analogWrite(Motor_1_PWM,0);
  pinMode(Motor_1_A,OUTPUT);
  digitalWrite(Motor_1_A,HIGH);
  pinMode(Motor_1_B,OUTPUT);
  digitalWrite(Motor_1_B,LOW);
  pinMode(Motor_1_Enable,OUTPUT);
  digitalWrite(Motor_1_Enable,HIGH);
  pinMode(Motor_1_D1,OUTPUT);
  digitalWrite(Motor_1_D1,LOW);
  
  pinMode(Motor_2_PWM,OUTPUT);
  analogWrite(Motor_2_PWM,0);
  pinMode(Motor_2_A,OUTPUT);
  digitalWrite(Motor_2_A,HIGH);
  pinMode(Motor_2_B,OUTPUT);
  digitalWrite(Motor_2_B,LOW);
  pinMode(Motor_2_Enable,OUTPUT);
  digitalWrite(Motor_2_Enable,HIGH);
  pinMode(Motor_2_D1,OUTPUT);
  digitalWrite(Motor_2_D1,LOW);
  
  pinMode(Motor_3_PWM,OUTPUT);
  analogWrite(Motor_3_PWM,0);
  pinMode(Motor_3_A,OUTPUT);
  digitalWrite(Motor_3_A,HIGH);
  pinMode(Motor_3_B,OUTPUT);
  digitalWrite(Motor_3_B,LOW);
  pinMode(Motor_3_Enable,OUTPUT);
  digitalWrite(Motor_3_Enable,HIGH);
  pinMode(Motor_3_D1,OUTPUT);
  digitalWrite(Motor_3_D1,LOW);
  //----- MOTORS -----//
  
  //----- CONTROLLER -----\\
  pinMode(CH6,INPUT); // Left Knob
  pinMode(CH5,INPUT); // Right Knob
  pinMode(CH4,INPUT); // Left-Right on Left Stick
  pinMode(CH3,INPUT); // Up-Down on Right Stick
  pinMode(CH2,INPUT); // Up-Down on Left Stick
  pinMode(CH1,INPUT); // Left-Right on Right Stick
  
  //CH2_Offset = pulseIn(CH2,HIGH); // Straight
  CH3_Offset = pulseIn(CH3,HIGH); // Right Motors
  CH4_Offset = pulseIn(CH4,HIGH); // Unused
  CH1_Offset = pulseIn(CH1,HIGH); // Strafe
  //----- CONTROLLER -----\\
}

void loop() {
  currentTime = millis();
  if ((currentTime - previousTime) >= calculatePIDInterval) {
    readController();
    calculatePID();
    previousTime = currentTime;
  }
}

void calculatePID() {
  //----- CALCULATE MOVING AVERAGE FOR EACH MOTOR -----\\
  Motor_1_MovingAverageFunction(); // Calculates the Moving Average for Motor 1
  Motor_2_MovingAverageFunction(); // Calculates the Moving Average for Motor 2
  Motor_3_MovingAverageFunction(); // Calculates the Moving Average for Motor 3
    
  movingAverageIndex += 1; // Increment the movingAverageIndex for the Moving Average
  if (movingAverageIndex == movingAverageSize) {
    movingAverageIndex = 0;
  } // If the movingAverageIndex is at the End of the Array, Reset movingAverageIndex
  //----- CALCULATE MOVING AVERAGE FOR EACH MOTOR -----\\
  
  //----- DETERMINE SETPOINT -----\\   
  robo_speed = sqrt((double)straight*(double)straight+(double)strafe*(double)strafe)*speed_gain;
  theta = (atan2(strafe,straight));//-90*3.14159/180);
  
  Setpoint[0] = (robo_speed*sin(theta-120*(3.14159/180)) + ( spin * spin_gain ));
  Setpoint[1] = (robo_speed*sin(theta)                   + ( spin * spin_gain ));
  Setpoint[2] = (robo_speed*sin(theta+120*(3.14159/180)) + ( spin * spin_gain ));
  //----- DETERMINE SETPOINT -----\\
  
  //----- DETERMINE INPUT -----\\
  if (Setpoint[0] < 0) {
    Motor_Right_Direction = -1; 
    Setpoint[0] = Setpoint[0] * -1;
  } else {
    Motor_Right_Direction = 1;
  }

  if (Setpoint[1] < 0) {
    Motor_Back_Direction = -1; 
    Setpoint[1] = Setpoint[1] * -1;
  } else {
    Motor_Back_Direction = 1;
  }
  if (Setpoint[2] < 0) {
    Motor_Left_Direction = -1; 
    Setpoint[2] = Setpoint[2] * -1;
  } else {
    Motor_Left_Direction = 1;
  }
  
  int previousMotorDirection[3] = {Motor_1_Direction,Motor_2_Direction,Motor_3_Direction};
  int newMotorDirection[3]      = {Motor_Right_Direction,Motor_Back_Direction,Motor_Left_Direction};
  double movingAverages[3]      = {Motor_1_MovingAverageValue,Motor_2_MovingAverageValue,Motor_3_MovingAverageValue};
  for (int i = 0; i < motorsRunning; i++) {
    if (previousMotorDirection[i] != newMotorDirection[i]) { // If the User is Switching the Direction of the Current Motor
      Input[i] = 0; // Start off at a Zero Velocity Since the Motor Will Be Reversed and Will Have a Start Velocity of Zero
    } else { // If the User is Not Switch the Direction of the Current Motor
      Input[i] = (equation_a*exp(equation_b*movingAverages[i]) + equation_c*exp(equation_d*movingAverages[i]) * equation_e); // Calculate Speed //  * PIDAdjust
      if ((Input[i] < equation_LowerLimit) || (movingAverages[i] == 0)) { // The equation used on works for a motor running about 25PWM
        Input[i] = 0;
      }
    }
  }
  //----- DETERMINE INPUT -----\\
 
  //----- CALCULATE PID -----\\
  if (Setpoint[0] != 0) {
    Motor_1_PID.Compute();
  } else {
    Output[0] = 0;
  }

  if (Setpoint[1] != 0) {
    Motor_2_PID.Compute();
  } else {
    Output[1] = 0;
  }

  if (Setpoint[2] != 0) {
    Motor_3_PID.Compute();
  } else {
    Output[2] = 0;
  }
  //----- CALCULATE PID -----\\
  
  //----- ADJUST OUTPUT-----\\
  for (int i = 0; i < motorsRunning; i++) {Output[i] = Output[i]/PIDAdjust;} // Reset PWM back to Non-Adjusted Value
  if (Output[0] < 0) {Motor_Right_Direction = -1;} else {Motor_Right_Direction = 1;}
  if (Output[1] < 0) {Motor_Back_Direction = -1;} else {Motor_Back_Direction = 1;}
  if (Output[2] < 0) {Motor_Left_Direction = -1;} else {Motor_Left_Direction = 1;}
  newMotorDirection[0] = Motor_Right_Direction;
  newMotorDirection[1] = Motor_Back_Direction;
  newMotorDirection[2] = Motor_Left_Direction;
  
  if ((spin == 0) && (straight == 0) && (strafe == 0)) {
    Output[0] = 0;
    Output[1] = 0;
    Output[2] = 0;
  }
  //----- ADJUST OUTPUT -----\\
  
  applyMotorDirections(previousMotorDirection,newMotorDirection); // Changes the Directions on Any Motors if Necessary
  
  //----- APPLY NEW PWM TO EACH MOTOR RESPECTIVELY -----\\
  analogWrite(Motor_1_PWM,Output[0]);
  analogWrite(Motor_2_PWM,Output[1]);
  analogWrite(Motor_3_PWM,Output[2]);
  //----- APPLY NEW PWM TO EACH MOTOR RESPECTIVELY -----\\
  
//  displayIndex++;
//  if (displayIndex == displayIterations) {
//    displayOutput(); // Display an Output of Calculated Data if Desired
//    displayIndex = 0;
//  }
}

void applyMotorDirections(int previousMotorDirection[3],int newMotorDirection[3]) {
  //----- IF MOTOR 1 DIRECTION CHANGED -----\\
  if (previousMotorDirection[0] != newMotorDirection[0]) {
    if (newMotorDirection[0] == 1) {
      digitalWrite(Motor_1_A,HIGH);
      digitalWrite(Motor_1_B,LOW);
    } else if (newMotorDirection[0] == -1) {
      digitalWrite(Motor_1_A,LOW);
      digitalWrite(Motor_1_B,HIGH);
    }
    Motor_1_Direction = newMotorDirection[0];
  }
  //----- IF MOTOR 1 DIRECTION CHANGED -----\\
  
  //----- IF MOTOR 2 DIRECTION CHANGED -----\\
  if (previousMotorDirection[1] != newMotorDirection[1]) {
    if (newMotorDirection[1] == 1) {
      digitalWrite(Motor_2_A,HIGH);
      digitalWrite(Motor_2_B,LOW);
    } else if (newMotorDirection[1] == -1) {
      digitalWrite(Motor_2_A,LOW);
      digitalWrite(Motor_2_B,HIGH);
    }
    Motor_2_Direction = newMotorDirection[1];
  }
  //----- IF MOTOR 2 DIRECTION CHANGED -----\\
  
  //----- IF MOTOR 3 DIRECTION CHANGED -----\\
  if (previousMotorDirection[2] != newMotorDirection[2]) {
    if (newMotorDirection[2] == 1) {
      digitalWrite(Motor_3_A,HIGH);
      digitalWrite(Motor_3_B,LOW);
    } else if (newMotorDirection[2] == -1) {
      digitalWrite(Motor_3_A,LOW);
      digitalWrite(Motor_3_B,HIGH);
    }
    Motor_3_Direction = newMotorDirection[2];
  }
  //----- IF MOTOR 3 DIRECTION CHANGED -----\\
}

void Motor_1_MovingAverageFunction() {
  //----- CALCULATE TICKS AND SPEED -----\\
  Motor_1_MotorRotations_CurrentSample = Motor_1_Rotations + (Motor_1_Ticks/ticksPerRevolution) - Motor_1_MotorRotations_LastSample;
  Motor_1_MotorRotations_LastSample = Motor_1_Rotations + (Motor_1_Ticks/ticksPerRevolution);
  Motor_1_MovingAverageMatrix[movingAverageIndex] = (Motor_1_MotorRotations_CurrentSample) / ((currentTime - previousTime)/milliPerMin);
  //----- CALCULATE TICKS AND SPEED -----\\
  
  //----- ERROR CHECKING -----\\
  if (Motor_1_MovingAverageMatrix[movingAverageIndex] > 500) { // In case the current average speed is infinity, set the current average speed as previous value
    if (movingAverageIndex == 1) {
      Motor_1_MovingAverageMatrix[movingAverageIndex] = Motor_1_MovingAverageMatrix[movingAverageSize];
    } else {
      Motor_1_MovingAverageMatrix[movingAverageIndex] = Motor_1_MovingAverageMatrix[movingAverageIndex-1];
    }
  }
  //----- ERROR CHECKING -----\\
  
  //----- CALCULATED MOVING AVERAGE VALUE -----\\
  Motor_1_MovingAverageValue = 0;
  for (int i = 0; i < movingAverageSize; i++) {
    Motor_1_MovingAverageValue +=  Motor_1_MovingAverageMatrix[i];
  }
  Motor_1_MovingAverageValue /= movingAverageSize;
  //----- CALCULATED MOVING AVERAGE VALUE -----\\
}

void Motor_2_MovingAverageFunction() {
  //----- CALCULATE TICKS AND SPEED -----\\
  Motor_2_MotorRotations_CurrentSample = Motor_2_Rotations + (Motor_2_Ticks/ticksPerRevolution) - Motor_2_MotorRotations_LastSample;
  Motor_2_MotorRotations_LastSample = Motor_2_Rotations + (Motor_2_Ticks/ticksPerRevolution);
  Motor_2_MovingAverageMatrix[movingAverageIndex] = (Motor_2_MotorRotations_CurrentSample) / ((currentTime - previousTime)/milliPerMin);
  //----- CALCULATE TICKS AND SPEED -----\\
  
  //----- ERROR CHECKING -----\\
  if (Motor_2_MovingAverageMatrix[movingAverageIndex] > 500) { // In case the current average speed is infinity, set the current average speed as previous value
    if (movingAverageIndex == 1) {
      Motor_2_MovingAverageMatrix[movingAverageIndex] = Motor_2_MovingAverageMatrix[movingAverageSize];
    } else {
      Motor_2_MovingAverageMatrix[movingAverageIndex] = Motor_2_MovingAverageMatrix[movingAverageIndex-1];
    }
  }
  //----- ERROR CHECKING -----\\
  
  //----- CALCULATED MOVING AVERAGE VALUE -----\\
  Motor_2_MovingAverageValue = 0;
  for (int i = 0; i < movingAverageSize; i++) {
    Motor_2_MovingAverageValue +=  Motor_2_MovingAverageMatrix[i];
  }
  Motor_2_MovingAverageValue /= movingAverageSize;
  //----- CALCULATED MOVING AVERAGE VALUE -----\\
}

void Motor_3_MovingAverageFunction() {
  //----- CALCULATE TICKS AND SPEED -----\\
  Motor_3_MotorRotations_CurrentSample = Motor_3_Rotations + (Motor_3_Ticks/ticksPerRevolution) - Motor_3_MotorRotations_LastSample;
  Motor_3_MotorRotations_LastSample = Motor_3_Rotations + (Motor_3_Ticks/ticksPerRevolution);
  Motor_3_MovingAverageMatrix[movingAverageIndex] = (Motor_3_MotorRotations_CurrentSample) / ((currentTime - previousTime)/milliPerMin);
  //----- CALCULATE TICKS AND SPEED -----\\
  
  //----- ERROR CHECKING -----\\
  if (Motor_3_MovingAverageMatrix[movingAverageIndex] > 500) { // In case the current average speed is infinity, set the current average speed as previous value
    if (movingAverageIndex == 1) {
      Motor_3_MovingAverageMatrix[movingAverageIndex] = Motor_3_MovingAverageMatrix[movingAverageSize];
    } else {
      Motor_3_MovingAverageMatrix[movingAverageIndex] = Motor_3_MovingAverageMatrix[movingAverageIndex-1];
    }
  }
  //----- ERROR CHECKING -----\\
  
  //----- CALCULATED MOVING AVERAGE VALUE -----\\
  Motor_3_MovingAverageValue = 0;
  for (int i = 0; i < movingAverageSize; i++) {
    Motor_3_MovingAverageValue +=  Motor_3_MovingAverageMatrix[i];
  }
  Motor_3_MovingAverageValue /= movingAverageSize;
  //----- CALCULATED MOVING AVERAGE VALUE -----\\
}

void readController() {
  //----- Get Inputs from Controller -----\\
  straight = pulseIn(CH3,HIGH) - CH3_Offset;
  spin = (pulseIn(CH4,HIGH) - CH4_Offset) * -1;
  strafe = (pulseIn(CH1,HIGH) - CH1_Offset) * -1;
  //----- Get Inputs from Controller -----\\
  
  //----- Adjust Controller Output -----\\
  if ((straight > -1*deadZone) && (straight < deadZone)) {
    straight = 0;
  }
  if ((spin > -1*deadZone) && (spin < deadZone)) {
    spin = 0;
  }
  if ((strafe > -1*deadZone) && (strafe < deadZone)) {
    strafe = 0;
  }
  //----- Adjust Controller Output -----\\
}

void displayOutput() {
  Serial.print("Controller: ");
  Serial.print(straight);
  Serial.print(" ");
  Serial.print(strafe);
  Serial.print(" ");
  Serial.print(spin);
  Serial.print("     ");
  
  Serial.print("Inputs: ");
  Serial.print(Input[0]);
  Serial.print(" ");
  Serial.print(Input[1]);
  Serial.print(" ");
  Serial.print(Input[2]);
  Serial.print("     ");
  
  Serial.print("Outputs: ");
  Serial.print(Output[0]);
  Serial.print(" ");
  Serial.print(Output[1]);
  Serial.print(" ");
  Serial.print(Output[2]);
  Serial.print("     ");
  
  Serial.print("Setpoints: ");
  Serial.print(Setpoint[0]);
  Serial.print(" ");
  Serial.print(Setpoint[1]);
  Serial.print(" ");
  Serial.println(Setpoint[2]);

  //Serial.println("");
}

void Motor_1_InterruptFunction() {
  Motor_1_Ticks += 1; // Increment Motor Ticks
  if (Motor_1_Ticks >= ticksPerRevolution) {
    Motor_1_Rotations += 1; Motor_1_Ticks -= ticksPerRevolution;
  } // If a full rotation has occurred, increment rotation and reset ticks
}

void Motor_2_InterruptFunction() {
  Motor_2_Ticks += 1; // Increment Motor Ticks
  if (Motor_2_Ticks >= ticksPerRevolution) {
    Motor_2_Rotations += 1; Motor_2_Ticks -= ticksPerRevolution;
  } // If a full rotation has occurred, increment rotation and reset ticks
}

void Motor_3_InterruptFunction() {
  Motor_3_Ticks += 1; // Increment Motor Ticks
  if (Motor_3_Ticks >= ticksPerRevolution) {
    Motor_3_Rotations += 1; Motor_3_Ticks -= ticksPerRevolution;
  } // If a full rotation has occurred, increment rotation and reset ticks
}


