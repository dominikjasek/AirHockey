// pulses needed for 1 rotation = 400 pulses/rot
// mm per rot = 60 * 2 = 120 mm/rot

// Constant duration of loop()
unsigned long t = 0;
#define CYCLE_DURATION 1800  //us

//Barriers
const uint8_t OFFSET_X = 12;
const byte OFFSET_Y = 11;
const float PUSHER_RADIUS = 47.5;
const float BARRIER_X_MIN  = 0 + PUSHER_RADIUS + OFFSET_X;
const float BARRIER_X_MAX = 447;
const float BARRIER_Y_MIN  = -300 + PUSHER_RADIUS + OFFSET_Y;
const float BARRIER_Y_MAX = 300 - PUSHER_RADIUS - OFFSET_Y;
const float START_Y = BARRIER_Y_MAX;
const float START_X = BARRIER_X_MIN;
const int INDUCTION_CONSTANT_SWITCH = 2; //due to current in motors, there is inductive current in microswitch and thus we are searching for 2 consecutive events when switch stays constant and doesnt jump
const int INDUCTION_DRIVER_SWITCH = 10; //due to current in motors, there is inductive current in microswitch and thus we are searching for 2 consecutive events when switch stays constant and doesnt jump

bool error = true;
bool error_drivers = false;
bool homing_state = false;
bool error_printed = false;
bool homed = false;

const float mmPerRev = 0.5 * 60 * 2; //60 teeth * 2mm teeth span
const float stepsPerRev =  400; //microstepping
const float HBOT_CONSTANT = mmPerRev/stepsPerRev; //mm->steps means divide, steps->mm means multiply (=0,15)

// Speed variables
float ACCEL_PER1SEC;  //accel for axis per 1 second
unsigned int DECEL_GAIN;
float DECEL;
float ACCEL;  //accel for motor in steps per cycle
float MM_SPEED;
float MAX_MOTOR_SPEED;
unsigned int Kp;

// Default values
const unsigned long ACCEL_PER1SEC_DEF = 20000; //acceleration per 1 second
const int MM_SPEED_DEF = 2000; //mm per second
const int Kp_DEF = 20; //P regulator
const int DECEL_GAIN_DEF = 1;

// Boundaries
#define MAX_ALLOWED_ACCEL 200
#define MAX_ALLOWED_ACCEL_PER1SEC 200000
#define MAX_MOTOR_ALLOWED_SPEED 20000
#define MAX_KPGAIN 1000


const unsigned int SPEED_TO_UPDATE = 0.3; //speed lower than this difference is ignored and not updated
const float POSITION_ERROR_TOLERANCE = 1.5; //must be greater than 0!!!

//Driver fault
#define DRIVER_FLT_0 A2
#define DRIVER_FLT_1 A3

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

// LED Strip
#define LED_STRIP 11

//switch interrupts
volatile bool switch_slider = false;
volatile bool switch_motor = false;
volatile bool switch_others = false;
volatile bool realSpeedsApplied = false;

// Solid-state relays
#define FANS_PIN A1
int fan_state = 0;
//#define SOLENOID_PIN A5
#define SOLENOID_PIN 9
const unsigned int SOLENOID_MIN_DELAY = 1000; //ms

// Lasers
#define GOAL_ROBOT 8
#define GOAL_HUMAN 10
#define GOAL_MIN_DELAY 4000
#define GOAL_SUM_AMOUNT 2

// Steppers pins
const int PUL1 = PD6; // Pul pin of stepper 1, Digital PIN 12
const int DIR1 = PD4; // cDir pin of stepper 1, Digital PIN 4
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

//Timer4 constants
#define RASPBERRY_DATA_LAG 5
#define OCR4B_value 100


//Position
volatile float pos_stepper[2] = {0,0};
volatile float pos_X = 0;
volatile float pos_Y = 0;

// Speed variables
bool preventWallHit = false;
bool preventWallHit_printed = false;
bool allowedSpeed[2] = {false,false};
bool changed[2] = {false,false};
const int initial_speed = 0; 
const unsigned int minimal_speed = 1; //minimal speed: 31 steps/sec for 8 prescaler
bool positionControl = false;
bool positionReached = false;
float desiredPosition[2] = {START_X,START_Y};
float desiredSpeed[2] = {0,0};
float realSpeedXY_mm[2] = {0,0};
float speedToCompare[2] = {0,0};
float realSpeed[2] = {0,0};
int direct[2] = {0, 0}; //1 = CCW, -1 = CW
//bool changing_dir[2] = {false, false};
//#define NOT_DEFINED 555
int dir_state[2];
