#include <math.h>

void demo() {
  square_path();
  circle_path();
}

void square_path() {
  waitUntilPosReached(400,-200);
  waitUntilPosReached(100,-200);
  waitUntilPosReached(100,200);
  waitUntilPosReached(400,200);
  waitUntilPosReached(400,-200);
  waitUntilPosReached(100,200);
  waitUntilPosReached(100,-200);
  waitUntilPosReached(400,200);
}


void circle_path() {
  const float RADIUS_MULTIPLIER = 0.8; // must be between 0 and 1
  #define POLYGON_CNT 12.0
  const float ANGLE_DELTA = 2*M_PI / POLYGON_CNT;
  //Serial.println("ANGLE_DELTA = " + String(ANGLE_DELTA));
  const float MID_X = (BARRIER_X_MAX + BARRIER_X_MIN)/2;
  const float MID_Y = 0;
  //Serial.println("MID_X, MID_Y = " + String(MID_X) + " " + String(MID_Y)); 
  const float radius = (MID_X-BARRIER_X_MIN)*RADIUS_MULTIPLIER;
  const float START_X = MID_X+radius;
  const float START_Y = MID_Y;
  //Serial.println("START_X, START_Y = " + String(START_X) + " " + String(START_Y)); 

  waitUntilPosReached(START_X,START_Y);

  for (int i = 0; i<500; i++) { // (!error)  {
    //Serial.println("Diff to mid point = " + String(pos_X-MID_X) + " " + String(pos_Y-MID_Y));
    double alpha = atan2(pos_Y-MID_Y,pos_X-MID_X);
    //Serial.println("alpha = " + String(alpha));
    double new_alpha = alpha + ANGLE_DELTA;
    //Serial.println("alpha = " + String(alpha));
    double des_X = radius*cos(new_alpha);
    double des_Y = radius*sin(new_alpha);
    //Serial.println("des_X+MID_X-pos_X, des_Y+MID_Y-pos_Y = " + String(des_X+MID_X) + " " + String(des_Y+MID_Y));
    setDesiredPosition(des_X+MID_X, des_Y+MID_Y);
    loop();
  }
  positionControl = false;
  setDesiredSpeedsMotors(0,0);
}
