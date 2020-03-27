void homing(bool& homing_state, bool& error, bool& positionControl)  {
  setZeroSpeeds();
  delay(10);
  setDefaultParams();
  if (!error_drivers) {
    detachInterrupts();
    int homing_speed = 700; 
    homing_state = true; switch_slider = false; 
    //bool prev_positionControl = positionControl;  //turn off positionControl for homing
    positionControl = false;
    
    if (switch_motor) { //pusher has hit motor switch, go forward a bit to prevent hitting corner of goal in case it is inside
      switch_motor = false;
      setNewDesiredSpeedsMotors(-homing_speed,homing_speed);
      error = false; 
      for (int i =0;i<1000;i++){
        updateRealSpeeds();  
      }
    }
    
    attachInterrupt(digitalPinToInterrupt(SWITCH_SLIDER_2), checkSwitchSlider2, LOW);
    //attachInterrupts();
    error = false; 
    Serial.println("Homing start");
    Serial.println("Homing Y axis");
    setNewDesiredSpeedsMotors(-homing_speed,-homing_speed);
    while(!switch_slider) {
      updateRealSpeeds();
    }  
    setZeroSpeeds();
      
    
    Serial.println("Homing X axis");
    setNewDesiredSpeedsMotors(homing_speed,-homing_speed);
    detachInterrupt(digitalPinToInterrupt(SWITCH_SLIDER_2));
    attachInterrupt(digitalPinToInterrupt(SWITCH_MOTOR), checkSwitchMotor, LOW);
    error = false;
    while(!switch_motor) {
      updateRealSpeeds(); 
    }
    
    setZeroSpeeds();
  
    resetPosition();  
    setNewDesiredSpeedsMotors(0,homing_speed);
    Serial.println("Moving to default position");
    error = false;
    delay(10);
    while (pos_X < 80){
      loop(); 
    }
    
    error = true;    
  
    setZeroSpeeds();
    //resetPosition();
    resetDesiredPosition();
    attachInterrupts();
    error = false; switch_motor = false; switch_slider = false; error_drivers = false;
    Serial.println("Homing finished");
    homing_state = false;
  }
  else  {
    Serial.println("There is an error in drivers. Please unplug the power from them, wait 5 seconds and try it again.");
  }
}
