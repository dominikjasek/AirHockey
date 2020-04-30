/*
  Author: Dominik Jašek, BUT BRNO 2020.
  MCU: Arduino Micro CPU: ATmega32U4
  Driver: EM705
  Stepper: 57HS22-A
  Microstepping: 400 steps/rev
  Datasheet of CPU: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf
  Pinmapping of CPU: https://www.arduino.cc/en/Hacking/PinMapping32u4
*/

#include "constants.h"

/*--------------------------------------------------------------------------------------*/
float mmToSteps(float mm) {
  return mm/HBOT_CONSTANT;
}

float stepsTomm(float steps)  {
  return steps*HBOT_CONSTANT;
}

void evaluatePos()  {
  pos_X = START_X + stepsTomm(-pos_stepper[0] + pos_stepper[1]);
  pos_Y = START_Y + stepsTomm(-pos_stepper[0] - pos_stepper[1]);
}

/*--------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);

  // Lasers
  //pinMode(13, OUTPUT); //diode for signalising goal
  pinMode(GOAL_ROBOT, INPUT);
  pinMode(GOAL_HUMAN, INPUT);

  // Relay
  pinMode(SOLENOID_PIN,OUTPUT);
  digitalWrite(SOLENOID_PIN, HIGH);
  pinMode(FANS_PIN,OUTPUT);
  digitalWrite(FANS_PIN, HIGH);
  
  // Led strip
  pinMode(LED_STRIP, OUTPUT);
  digitalWrite(LED_STRIP, LOW);

  // set end switches as input
  pinMode(SWITCH_SLIDER_2, INPUT_PULLUP);
  pinMode(SWITCH_MOTOR, INPUT_PULLUP);
  pinMode(SWITCH_OTHERS, INPUT_PULLUP);
  pinMode(DRIVER_FLT_0, INPUT);
  pinMode(DRIVER_FLT_1, INPUT);
  //pinMode(GOAL_SENSOR_ROBOT,INPUT);
  attachInterrupts();
  switch_slider = false; switch_others = false;

  //set PUL1 and DIR1 to be output  --MOTOR1--
  DDRD |= (1 << PUL1);
  DDRD |= (1 << DIR1);
  //set PUL2 and DIR2 to be output  --MOTOR2--
  DDRC |= (1 << PUL2);
  DDRD |= (1 << DIR2);

  //resetDirections();

  //Reset Timer1 Control Reg A
  TCCR1A = 0;
  //Reset Timer3 Control Reg A
  TCCR3A = 0;

  // Set CTC mode on Timer1, it ensures that each time comp triggers, it automatically resets the timer to 0
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);
  // Set CTC mode on Timer3, it ensures that each time comp triggers, it automatically resets the timer to 0
  TCCR3B &= ~(1 << WGM13);
  TCCR3B |= (1 << WGM12);
  
  //Set to prescaler of 8
  TCCR1B &= ~(1 << CS12);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS10);
  //Set to prescaler of 8
  TCCR3B &= ~(1 << CS12);
  TCCR3B |= (1 << CS11);
  TCCR3B &= ~(1 << CS10);

  //Reset Timer1 on Motor 2 and set compare value
  TCNT1 = 0;
  //OCR1A = stepsToComp(initial_speed);
  //Reset Timer3 on Motor 1 and set compare value
  TCNT3 = 0;
  //OCR3A = stepsToComp(initial_speed);

  //Enable Timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);
  //Enable Timer3 compare interrupt
  TIMSK3 = (1 << OCIE3A);

  //TIMER 4 - high speed 10-bit timer - check goal and send data to Raspberry, see datasheet page 168
  //Set to prescaler of 512, Timer4 has frequency 64MHz
  //tick lasts 100/(64000000/prescaler) us = 100/(64000000/512) = 0.8ms and thus 125x times per second, sending data is 10x longer
  TCCR4B |= (1 << CS43);
  TCCR4B &= ~(1 << CS42);
  TCCR4B |= (1 << CS41);
  TCCR4B &= ~(1 << CS40);
  TCNT4 = 0;  //set counter to 0
  OCR4B = OCR4B_value;
  //OCR4A = OCR4A_value;  // update realSpeed every 1ms
  TIMSK4 |= (1 << OCIE4B);   //allow timer4 interrupt
  //TIMSK4 |= (1 << OCIE4A);   //allow timer4 interrupt

  //Enable global interrupts
  sei();

  setZeroSpeeds();
  delay(50);  

  setDefaultParams();
  delay(2000); 
  Serial.println("restarted");
}

/*--------------------------------------------------------------------------------------*/

void loop() {
  //Serial.println("FLT_0 = " + String(digitalRead(DRIVER_FLT_0)) + " FLT_1 = " + String(digitalRead(DRIVER_FLT_1)));
  checkSerialInput();
  evaluatePos();
  checkDriverError();
  if (!error) {
    updateRealSpeeds();
  } 
  //Serial.println(micros()-t2);  
  if (micros()-t >= CYCLE_DURATION)  {
    Serial.println("Cycle took longer time than cycle_duration: " + String(micros()-t));
  }
  else {
    while(micros()-t < CYCLE_DURATION){}  //wait for cycle to be time-equidistant 
  }
  applyRealSpeeds();
  t = micros();
}
