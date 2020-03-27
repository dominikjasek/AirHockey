void demo() {

  const int X_END = 300;
  const int X_START = 190;
  const int Y_END = -70;
  const int Y_START = 70;
  const int max_speed = MAX_SPEED; // max 8200

  setNewDesiredSpeedsMotors(0,200);
  print_pos();
  while (pos_X < 100) {
      updateRealSpeeds();
    if (!checkBarriers()) {
      setZeroSpeeds();
    }
    evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
  }

  for (int i = 0; i<50 ;i++)  {
    Serial.print("Iteration: ");
    Serial.println(i);
    setNewDesiredSpeedsMotors(-max_speed,max_speed);
    while (pos_X < X_END && !error) {
      updateRealSpeeds();
      if (!checkBarriers()) {
        Serial.println("LINE 1");
        print_real_speeds();
        print_pos();
        print_steps();
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }
    /*Serial.println("1st line finished----------------------------------------------------");
    print_pos();
    Serial.println("---------------------------------------------------------------------");*/
    
    setNewDesiredSpeedsMotors(max_speed,max_speed);
    while (pos_Y > Y_END && !error) {
      updateRealSpeeds();
      if (!checkBarriers()) {
        Serial.println("LINE 2");
        print_real_speeds();
        print_pos();
        print_steps();
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }
    /*Serial.println("2nd line finished----------------------------------------------------");
    print_pos();
    Serial.println("---------------------------------------------------------------------");*/

    setNewDesiredSpeedsMotors(max_speed,-max_speed);
    while (pos_X > X_START && !error) {
      updateRealSpeeds();
      if (!checkBarriers()) {
        Serial.println("LINE 3");
        print_real_speeds();
        print_pos();
        print_steps();
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }
    /*Serial.println("3rd line finished----------------------------------------------------");
    print_pos();
    Serial.println("---------------------------------------------------------------------");*/
  
    setNewDesiredSpeedsMotors(-max_speed,-max_speed);
    while (pos_Y < Y_START && !error) {
      updateRealSpeeds();
      if (!checkBarriers()) {
        Serial.println("LINE 4");
        print_real_speeds();
        print_pos();
        print_steps();
        setZeroSpeeds();
      }
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
    }
    /*Serial.println("4th line finished----------------------------------------------------");
    print_pos();
    Serial.println("---------------------------------------------------------------------");*/
  }
  resetDesiredPosition();
  setNewDesiredSpeedsMotors(0,0);
  Serial.println("Demo finished");
}


bool checkBarriers()  {
  int delta0 = 0;
  int delta1 = 0;
  if (realSpeed[0] == 0)  {
    delta0 = sgn(desiredSpeed[0]);
  }
  else  {
    delta0 = sgn(realSpeed[0]);
  }
  if (realSpeed[1] == 0)  {
    delta1 = sgn(desiredSpeed[1]);
  }
  else  {
    delta1 = sgn(realSpeed[1]);
  }
  return true;
  //evaluatePos(pos_stepper[0] + 50*delta0 , pos_stepper[1] + 50*delta1, pos_help_X, pos_help_Y);
  //if (BARRIER_X_MIN <= pos_help_X && pos_help_X <= BARRIER_X_MAX && BARRIER_Y_MIN <= pos_help_Y &&  pos_help_Y <= BARRIER_Y_MAX) {
  //  return true;
  //}
  //else  {
  //  //Serial.println("This move is not possible..." + String( pos_help_X) + ", " + String( pos_help_Y));
  //  return false;
  //}
}
