void loopWithoutSerial()  {
  evaluatePos();
  checkDriverError();
  if (!error) {
    updateRealSpeeds();
  } 
  while((micros()-t)<CYCLE_DURATION){}  //wait
  applyRealSpeeds();
  t = micros();
}

void homing()  {
  const float mm_speed_restore = MM_SPEED;
  const float accel_per1sec_restore = ACCEL_PER1SEC; 
  //Serial.println("mm_speed_restore = " + String(mm_speed_restore));
  //Serial.println("accel_restore = " + String(accel_per1sec_restore));
  const int homing_speed = 150; //mm/s
  const int accelper1sec = 1200; //mm/s^2
  setZeroSpeeds();
  delay(10);
  setMaximalSpeed(homing_speed);
  setAccel(accelper1sec);
  if (!error_drivers) {
    detachInterrupts();
    
    homing_state = true; switch_slider = false; 
    //bool prev_positionControl = positionControl;  //turn off positionControl for homing
    positionControl = false;
    
    if (switch_motor == true) { //pusher has hit motor switch, go forward a bit to prevent hitting corner of goal in case it is inside
      switch_motor = false;
      setDesiredSpeedsXY(homing_speed,0);
      error = false; 
      for (int i =0;i<150;i++){
        loopWithoutSerial();
      }
    }
    
    attachInterrupt(digitalPinToInterrupt(SWITCH_SLIDER_2), checkSwitchSlider2, LOW);
    //attachInterrupts();
    error = false; 
    Serial.println("Homing start");
    Serial.println("Homing Y axis");
    setDesiredSpeedsXY(0,homing_speed);
    while(!switch_slider) {
      loopWithoutSerial();
    }  
    setZeroSpeeds();
    pos_stepper[0] = -467;
    pos_stepper[1] = 468;
      
    
    Serial.println("Homing X axis");
    setDesiredSpeedsXY(-homing_speed,0);
    detachInterrupt(digitalPinToInterrupt(SWITCH_SLIDER_2));
    attachInterrupt(digitalPinToInterrupt(SWITCH_MOTOR), checkSwitchMotor, LOW);
    error = false;
    while(!switch_motor) {
      loopWithoutSerial();
    }
    
    setZeroSpeeds();
  
    resetPosition();  
    setDesiredSpeedsXY(homing_speed/2,-homing_speed/2);
    Serial.println("Moving to default position");
    error = false;
    delay(10);
    while (pos_X < 80){
      loopWithoutSerial();
    }    
    //error = true;      
    setZeroSpeeds();
    setMaximalSpeed(mm_speed_restore);
    setAccel(accel_per1sec_restore);
    resetDesiredPosition();
    attachInterrupts();
    error = false; switch_motor = false; switch_slider = false; switch_others = false; error_drivers = false;
    //Serial.println("Homing finished");
    homing_state = false;
    homed = true;
    sendDataToRaspberry(true);
  }
  else  {
    Serial.println("There is an error in drivers. Please unplug the power from them, wait 5 seconds and try it again.");
  }
}
