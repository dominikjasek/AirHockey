/*
  Author: Dominik Jašek, BUT BRNO 2020.
  MCU: Arduino Micro CPU: ATmega32U4
  Driver: EM705
  Stepper: 57HS22-A
  Microstepping: 400 steps/rev
  Datasheet of CPU: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf
  Pinmapping of CPU: https://www.arduino.cc/en/Hacking/PinMapping32u4
*/

// pulses needed for 1 rotation = 400 pulses/rot
// mm per rot = 60 * 2 = 120 mm/rot

#include "Configuration.h"

// Constant duration of loop()
unsigned long t = 0;
#define CYCLE_DURATION 2100  //us

//Barriers
const byte OFFSET_X = 12;
const byte OFFSET_Y = 11;
const float PUSHER_RADIUS = 47.5;
const float BARRIER_X_MIN  = 0 + PUSHER_RADIUS + OFFSET_X;
const float BARRIER_X_MAX = 447;
const float BARRIER_Y_MIN  = -300 + PUSHER_RADIUS + OFFSET_Y;
const float BARRIER_Y_MAX = 300 - PUSHER_RADIUS - OFFSET_Y;
const float START_Y = BARRIER_Y_MAX;
const float START_X = BARRIER_X_MIN;
const int INDUCTION_CONSTANT_SWITCH = 2; //due to current in motors, there is inductive current in microswitch and thus we are searching for 2 consecutive events when switch stays constant and doesnt jump

bool error = true;
bool error_drivers = false;
bool homing_state = false;
bool error_printed = false;


const float mmPerRev = 0.5 * 60 * 2; //60 teeth * 2mm teeth span
const float stepsPerRev =  400; //microstepping
const float HBOT_CONSTANT = mmPerRev/stepsPerRev; //mm->steps means divide, steps->mm means multiply

// Speed variables
float ACCEL_PER1SEC;  //accel for axis per 1 second
float ACCEL;  //accel for motor in steps per cycle
float MM_SPEED;
float MAX_MOTOR_SPEED;
unsigned int Kp;

// Default values
const unsigned long ACCEL_PER1SEC_DEF = 30000; //acceleration per 1 second
const int MM_SPEED_DEF = 3000; //mm per second
const int Kp_DEF = 10; //P regulator

// Boundaries
#define MAX_ALLOWED_ACCEL 200
#define MAX_ALLOWED_ACCEL_PER1SEC 200000
#define MAX_MOTOR_ALLOWED_SPEED 20000
#define MAX_KPGAIN 1000


//const unsigned int SPEED_IN_TOLERANCE = ACCEL+1;
const float POSITION_ERROR_TOLERANCE = 0.4; //must be greater than 0!!!

//Driver fault
#define DRIVER_FLT_0 9  //PB5
#define DRIVER_FLT_0_PIN PINB
#define DRIVER_FLT_0_REGISTER_NUM 5

#define DRIVER_FLT_1 10  //PB6
#define DRIVER_FLT_1_PIN PINB
#define DRIVER_FLT_1_REGISTER_NUM 6

// End switches
#define SWITCH_SLIDER_2 7 //PE6
#define SWITCH_SLIDER_2_PIN PINE
#define SWITCH_SLIDER_2_REGISTER_NUM 6

#define SWITCH_MOTOR 3 //PD0
#define SWITCH_MOTOR_PIN PIND
#define SWITCH_MOTOR_REGISTER_NUM 0

#define SWITCH_OTHERS 2  //PD1
#define SWITCH_OTHERS_PIN PIND
#define SWITCH_OTHERS_REGISTER_NUM 1  

// Goal laser sensors
#define GOAL_SENSOR_ROBOT 14  //PF7, A0 Pin
#define GOAL_SENSOR_ROBOT_PIN PINF
#define GOAL_SENSOR_ROBOT_REGISTER_NUM 7

//switch interrupts
volatile bool switch_slider = false;
volatile bool switch_motor = false;
volatile bool switch_others = false;
volatile bool realSpeedsApplied = false;


// Steppers pins
const int PUL1 = PD6; // Pul pin of stepper 1, Digital PIN 12
const int DIR1 = PD4; // Dir pin of stepper 1, Digital PIN 4
const int PUL2 = PC6; // Pul pin of stepper 2, Digital PIN 5
const int DIR2 = PD7; // Dir pin of stepper 2, Digital PIN 6

// Timer Interrupt Compare variables
#define TIMER_LIMIT 65535
const int prescaler = 8; // if you change this value, you need to change TCCR1B and TCCR3B too!!!
const long COMP_CONSTANT = 16000000/prescaler;
volatile int Tim1_count = 0;  //counter to go above highest value to compare
int Tim1_multiplier = 0;  //used to compare Tim1_count if minimal speed is too low for one comp cycle
uint16_t Tim1_res_comp = 0;
volatile int Tim3_count = 0;
int Tim3_multiplier = 0;
uint16_t Tim3_res_comp = 0;


//Position
volatile float pos_stepper[2] = {0,0};
volatile float pos_X = 0;
volatile float pos_Y = 0;

// Speed variables
bool allowedSpeed[2] = {false,false};
bool changed[2] = {false,false};
const int initial_speed = 0; 
const int minimal_speed = 0; //minimal speed: 31 steps/sec for 8 prescaler
bool positionControl = false;
bool positionReached = false;
float desiredPosition[2] = {START_X,START_Y};
float desiredSpeed[2] = {0,0};
float realSpeedXY_mm[2] = {0,0};
float speedToCompare[2] = {0,0};
float realSpeed[2] = {0,0};
int direct[2] = {0, 0}; //1 = CCW, -1 = CW
int lastdirect[2] = {555, 555};

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

  pinMode(13, OUTPUT);

  // set end switches as input
  pinMode(SWITCH_SLIDER_2, INPUT_PULLUP);
  pinMode(SWITCH_MOTOR, INPUT_PULLUP);
  pinMode(SWITCH_OTHERS, INPUT_PULLUP);
  pinMode(DRIVER_FLT_0, INPUT);
  pinMode(DRIVER_FLT_1, INPUT);
  pinMode(GOAL_SENSOR_ROBOT,INPUT);
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

  //TIMER 4 - send data to Raspberry, see datasheet page 168
  //Set to prescaler of 512, Timer4 has frequency 64MHz
  //tick lasts 1/(64000000/prescaler) us = 1/(64000000/512) = 8us and thus 125x times per second
  TCCR4B |= (1 << CS43);
  TCCR4B &= ~(1 << CS42);
  TCCR4B |= (1 << CS41);
  TCCR4B &= ~(1 << CS40);
  TCNT4 = 0;  //set counter to 0
  OCR4A = 1000;  // update realSpeed every 1ms
  TIMSK4 = (1 << OCIE4A);   //allow time4 interrupt

  //Enable global interrupts
  sei();

  setZeroSpeeds();
  delay(100);  

  setDefaultParams();
  delay(100);  
  Serial.println("Setup finished");
}

/*--------------------------------------------------------------------------------------*/

void loop() {  
  //print_real_speedsXY();
  //Serial.println("TCNT3 type is " + String(TCNT1);
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
  while(micros()-t < CYCLE_DURATION){}  //wait for cycle to be time-equidistant 
  applyRealSpeeds();
  t = micros();
}
