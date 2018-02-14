/** 
 * @file run_motor_simple.ino
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2017-01-20
 */

const int PIN_A   = 2;
const int PIN_B   = 3;
const int PIN_PWM = 4;

void moveCounterClockwise(void);
void moveClockwise(void);
void changeSpeed(int);

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
}

void loop() {
  changeSpeed(150);
  moveCounterClockwise();
  delay(1000);
  changeSpeed(255);
  moveClockwise();
  delay(1000);
}

void moveCounterClockwise(void) {
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, HIGH);
}

void moveClockwise(void) {
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, LOW);
}

void changeSpeed(int motor_speed) {
  analogWrite(PIN_PWM, motor_speed);
}


