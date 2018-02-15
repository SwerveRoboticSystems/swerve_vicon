/** 
 * @file rc_comm.ino
 * @breif Example of how to use timer 1 as an ISR at 1Hz on the Arduino Uno
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2017-02-14
 */

#define PULSEIN_TIMEOUT 100000 // 100ms timeout

const int PIN_CH1 = 9;
const int PIN_CH2 = 10;
const int PIN_CH3 = 11;

int ch_1_value = 0; 
int ch_2_value = 0;
int ch_3_value = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);
  pinMode(PIN_CH3, INPUT);
  
  cli(); // stop interrupts from occurring to change settings

  // Setup Timer 1 on Arduino for Interrupt at 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  OCR1A = 15624; // set compare match register for 1hz increments, = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  sei(); // allow interrupts to execute

}

ISR(TIMER1_COMPA_vect){ // timer1 interrupt 1Hz
  ch_1_value = pulseIn(PIN_CH1, HIGH, PULSEIN_TIMEOUT);
  ch_2_value = pulseIn(PIN_CH2, HIGH, PULSEIN_TIMEOUT);
  ch_3_value = pulseIn(PIN_CH3, HIGH, PULSEIN_TIMEOUT);
  Serial.println("Executed interrupt");
}

void loop() {
  Serial.print("CH1: ");
  Serial.print(ch_1_value);
  Serial.print(" | CH2: ");
  Serial.print(ch_2_value);
  Serial.print(" | CH3: ");
  Serial.println(ch_3_value);

  delay(250);
}



