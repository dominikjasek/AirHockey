void calibrate()  {
  
  int calibration_speed = 300;
  int offset = PUSHER_RADIUS + 6;
  delay(2000);
  Serial.println("Starting calibration");
  Serial.println("Calibrating Y axis");

  int pressed = 0;

  setNewDesiredSpeeds(-calibration_speed,-calibration_speed);
  while(true) {
    updateSpeeds();
    if (!digitalRead(SWITCH_SLIDER))  {
      Serial.println("Motor Switch pressed");
      if (++pressed == 3)  {
        break;
      }
    }
    else  {
      //Serial.println("not pressed");
      pressed=0;
    }
  }

  pressed = 0;
  
  setZeroSpeeds();
  pressed = 0;
  delay(100);
  setNewDesiredSpeeds(calibration_speed,-calibration_speed);
  Serial.println("Calibrating X axis");
  
  while(true) {
    updateSpeeds();
    if (!digitalRead(SWITCH_MOTOR))  {
      Serial.println("Motor Switch pressed");
      if (++pressed == 3)  {
        break;
      }
    }
    else  {
      //Serial.println("not pressed");
      pressed=0;
    }
  }
  
  setZeroSpeeds();
  


  setNewDesiredSpeeds(0,calibration_speed/2);
  Serial.println("Moving to default position");
  pos_X = 0; pos_Y = 0; pos_help_X = 0; pos_help_Y = 0; pos_stepper[0] = 0; pos_stepper[1] = 0;
  while (pos_X < offset)  {
    Serial.println(pos_X);
     updateSpeeds();
     evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
  }
  setZeroSpeeds();
  pos_X = 0; pos_Y = 0; pos_help_X = 0; pos_help_Y = 0; pos_stepper[0] = 0; pos_stepper[1] = 0;
  
  Serial.println("Calibration finished");
  delay(100);
}



/* ------------------------------------------------------------------------------------------------------- */


void demo() {

  const unsigned int X_END = 0.8*BARRIER_X_MAX;
  const unsigned int X_START = 0.2*BARRIER_X_MAX;
  const unsigned int Y_END = 0.8*BARRIER_Y_MIN;
  const unsigned int Y_START = 0.8*BARRIER_Y_MAX;

  setNewDesiredSpeeds(0,200);
  print_pos();
  while (pos_X < 100) {
    updateSpeeds();
    if (!checkBarriers()) {
      setZeroSpeeds();
    }
    evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
  }

  for (int i = 0; i<5 ;i++)  {
    print_pos();
    setNewDesiredSpeeds(-MAX_SPEED,MAX_SPEED);
    while (pos_X < X_END) {
      updateSpeeds();
      if (!checkBarriers()) {
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }
    
    setNewDesiredSpeeds(MAX_SPEED,MAX_SPEED);
    print_pos();
    while (pos_Y > Y_END) {
      updateSpeeds();
      if (!checkBarriers()) {
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }

//    setNewDesiredSpeeds(0,-MAX_SPEED);
//    print_pos();
//    while (pos_X > X_START) {
//      updateSpeeds();
//      if (!checkBarriers()) {
//        setZeroSpeeds();
//      }
//      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
//    }

//    setNewDesiredSpeeds(MAX_SPEED,MAX_SPEED);
//    print_pos();
//    while (pos_Y < Y_END) {
//      updateSpeeds();
//      if (!checkBarriers()) {
//        setZeroSpeeds();
//      }
//      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
//    }
//
//    setNewDesiredSpeeds(-MAX_SPEED,0);
//    print_pos();
//    while (pos_X < X_END) {
//      updateSpeeds();
//      if (!checkBarriers()) {
//        setZeroSpeeds();
//      }
//      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
//    }

//    setNewDesiredSpeeds(MAX_SPEED,MAX_SPEED);
//    print_pos();
//    while (pos_Y < Y_END) {
//      updateSpeeds();
//      if (!checkBarriers()) {
//        setZeroSpeeds();
//      }
//      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
//    }
  
    setNewDesiredSpeeds(MAX_SPEED,-MAX_SPEED);
    print_pos();
    while (pos_X > X_START) {
      updateSpeeds();
      if (!checkBarriers()) {
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }
  
    setNewDesiredSpeeds(-MAX_SPEED,-MAX_SPEED);
    print_pos();
    while (pos_Y < Y_START) {
      updateSpeeds();
      if (!checkBarriers()) {
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }
  }

  setNewDesiredSpeeds(0,-200);
  print_pos();
  while (pos_X > 2) {
    updateSpeeds();
    if (!checkBarriers()) {
      setZeroSpeeds();
    }
    evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
  }

  setNewDesiredSpeeds(-200,-200);
  print_pos();
  while (pos_Y > 2) {
    updateSpeeds();
    if (!checkBarriers()) {
      setZeroSpeeds();
    }
    evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
  }
  
  setNewDesiredSpeeds(0,0);
}
