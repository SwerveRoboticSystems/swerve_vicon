/** 
 * @file rc_motor_control.ino
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2017-01-27
 */

#define CH1_MAX 1640
#define CH1_MID 1470
#define CH1_MIN 1300
#define CH2_MAX 1350
#define CH2_MIN 1700

const int CH1 = 10;
const int CH2 = 11;
int CH1_OFFSET, CH2_OFFSET;

const int PWM_PIN_1  = 3;
const int IN_A_PIN_1 = 2;
const int IN_B_PIN_1 = 1;

int moveMotor(int);

void setup() {
  
  Serial.begin(9600);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);

  pinMode(PWM_PIN_1, OUTPUT);
  pinMode(IN_A_PIN_1, OUTPUT);
  pinMode(IN_B_PIN_1, OUTPUT);

  analogWrite(PWM_PIN_1, 0);
  digitalWrite(IN_A_PIN_1, LOW);
  digitalWrite(IN_B_PIN_1, LOW);

  Serial.println("Waiting for controller connection");
  while (pulseIn(CH1, HIGH, 100000) == 0) {
    delay(100);
  }
  Serial.println("Controller connection received");
  

  CH1_OFFSET = pulseIn(CH1, HIGH, 100000);
  CH2_OFFSET = pulseIn(CH2, HIGH, 100000);
  
}

void loop() {
  
  int channel_1 = pulseIn(CH1, HIGH, 100000);
  int channel_2 = pulseIn(CH2, HIGH, 100000);

  moveMotor(channel_1, 1);
  
  Serial.print(" | CH1: ");
  Serial.print(channel_1);
  Serial.print(" | CH2: ");
  Serial.println(channel_2);

  delay(100);
  
}

int moveMotor(int input, int channel) {

  if (channel == 1) {
    
    if (input > CH1_MID) {
      digitalWrite(IN_A_PIN_1, HIGH);
      digitalWrite(IN_B_PIN_1, LOW);
      Serial.print("  CW    ");
    } else if (input < CH1_MID) {
      digitalWrite(IN_A_PIN_1, LOW);
      digitalWrite(IN_B_PIN_1, HIGH);
      Serial.print("  CCW   ");
    } else {
      digitalWrite(IN_A_PIN_1, LOW);
      digitalWrite(IN_B_PIN_1, LOW);
      Serial.print("  ELSE  ");
    }

    int motor_speed = round(abs(input - CH1_MID) * (255.0/170.0));

    if (motor_speed > 255) {
      motor_speed = 255;
    } else if (motor_speed < 0) {
      motor_speed = 0;
    }
    
    analogWrite(PWM_PIN_1, motor_speed);
    Serial.print("Motor Speed: ");
    Serial.print(motor_speed);
    
  }
  
}



