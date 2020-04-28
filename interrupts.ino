void checkGoal()  {
  static unsigned long lastGoalTimestamp = 0;
  //Serial.println("checking goal");
  if (millis()-lastGoalTimestamp > GOAL_MIN_DELAY)  {
    if (digitalRead(GOAL_ROBOT) == LOW) {
      Serial.println("gr");
      //tohle delalo posranej error
      /*while (digitalRead(GOAL_ROBOT) == LOW)  {
        Serial.println(GOAL_ROBOT);
      };   */   
      lastGoalTimestamp = millis();
      //digitalWrite(13,HIGH);
      //delay(3000);
      //digitalWrite(13,LOW);      
    }
    if (digitalRead(GOAL_HUMAN) == LOW) {
      Serial.println("gh");
      //tohle delalo posranej error
      /*while (digitalRead(GOAL_HUMAN) == LOW)  {
        Serial.println(GOAL_HUMAN);
      }; */      
      lastGoalTimestamp = millis();
      //digitalWrite(13,HIGH);
      //delay(3000);
      //digitalWrite(13,LOW);
    }
  }
}

void errorTrigger()  {
  error = true;
  error_printed = false;
  homed = false;
  //Serial.println("ERROR TRIGGER");
  //print_pos();
  setZeroSpeeds();
  detachInterrupts();
}

void checkDriverError() {  
  if (!(digitalRead(DRIVER_FLT_0)) || !(digitalRead(DRIVER_FLT_1))) {
    int i = 0;
    while (!(digitalRead(DRIVER_FLT_0)) || !(digitalRead(DRIVER_FLT_1))) {  // && !error_drivers
      if (++i >= INDUCTION_CONSTANT_SWITCH)  {
        error_drivers = true;
        errorTrigger();
        //Serial.println("Driver error...");
        return;
      }
    }
  }
  else  {
    error_drivers = false;
  }
}

void attachInterrupts()  {
  attachInterrupt(digitalPinToInterrupt(SWITCH_SLIDER_2), checkSwitchSlider2, LOW);
  attachInterrupt(digitalPinToInterrupt(SWITCH_MOTOR), checkSwitchMotor, LOW);
  attachInterrupt(digitalPinToInterrupt(SWITCH_OTHERS), checkSwitchOthers, LOW);
}

void detachInterrupts() {
  detachInterrupt(digitalPinToInterrupt(SWITCH_SLIDER_2));
  detachInterrupt(digitalPinToInterrupt(SWITCH_MOTOR));
  detachInterrupt(digitalPinToInterrupt(SWITCH_OTHERS));
}

void checkSwitchSlider2()  {
  int i = 0;
  while (!bitRead(SWITCH_SLIDER_2_PIN,SWITCH_SLIDER_2_REGISTER_NUM) && !switch_slider ) {
    //Serial.println("slider");
    if (++i >= INDUCTION_CONSTANT_SWITCH)  {
      //Serial.println("slider2 switch");
      switch_slider = true;
      errorTrigger();
      return;
    }
  }
}

void checkSwitchMotor()  {
  int i = 0;
  while (!bitRead(SWITCH_MOTOR_PIN,SWITCH_MOTOR_REGISTER_NUM) && !switch_motor) {
    //Serial.println("motor");
    if (++i >= INDUCTION_CONSTANT_SWITCH)  {
      //Serial.println("motor switch");
      switch_motor = true;
      errorTrigger();
      return;
    }
  }
}

void checkSwitchOthers()  {
  int i = 0;
  //Serial.println("checkSwitchOthers, value = " +String(bitRead(SWITCH_OTHERS_PIN,SWITCH_OTHERS_REGISTER_NUM)));
  while (!bitRead(SWITCH_OTHERS_PIN,SWITCH_OTHERS_REGISTER_NUM) && !switch_others ) {
    //Serial.println("others i = " + String(i));
    if (++i >= INDUCTION_CONSTANT_SWITCH)  {
      //Serial.println("others switch");
      switch_others = true;
      errorTrigger();
      return;
    }
  }
}

ISR(TIMER1_COMPA_vect)  { //Timer for motor 1
  //Serial.println("TIMER1_COMPA_vect, OCR1A = "+ String(OCR1A) +" Tim1_count = " + String(Tim1_count) + " Tim1_multiplier = " + String(Tim1_multiplier));
  if (direct[1] == 0 || error)
    return;

  if(Tim1_count >= Tim1_multiplier)  {
    //SET(PORTC, PUL2); // STEP X-AXIS (MOTOR1)
    PORTC|=(1<<PUL2);
    pos_stepper[1] += direct[1];
    __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");  // Wait for step pulse
    //CLR(PORTC, PUL2); CLR(x,y) (x&=(~(1<<y))) 
    PORTC&=(~(1<<PUL2));
    Tim1_count = 0;
    OCR1A = (Tim1_multiplier == 0)? (Tim1_res_comp) : (65535);
    return;
  }

  else {
    if(++Tim1_count == Tim1_multiplier)
      OCR1A = Tim1_res_comp;
      return;
  }
  
}

ISR(TIMER3_COMPA_vect)  { //Timer for motor 0
  if (direct[0] == 0 || error)
    return;
  
  if(Tim3_count >= Tim3_multiplier)  {
    //SET(PORTD, PUL1); // STEP X-AXIS (MOTOR1)
    PORTD|=(1<<PUL1);
    pos_stepper[0] += direct[0];
    __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");  // Wait for step pulse
    //CLR(PORTD, PUL1); 
    PORTD&=(~(1<<PUL1));
    Tim3_count = 0;
    OCR3A = (Tim3_multiplier == 0)? (Tim3_res_comp) : (65535);
    return;
  }
  
  else {
    //Tim3_count++;
    if(++Tim3_count == Tim3_multiplier)   { 
      OCR3A = Tim3_res_comp;     
    } 
  }
}

void sendDataToRaspberry(bool enforced)  { //Timer for sending serial data
  static int sent[5] = {-1,0,0,0,0};
  interrupts (); //alow other (motor) interrupts
  if (error && !error_printed && !homing_state)  {
    if (error_drivers) {
      //Serial.println("e2");
    }
    else {
      Serial.println("e1");
    }
    error_printed = true;
  }
  else if ((int)homed != sent[0] ||(int)pos_X != sent[1] || (int)pos_Y != sent[2] || (int)realSpeedXY_mm[0] != sent[3] || (int)realSpeedXY_mm[1] != sent[4] || enforced) {
    sent[0] = (int)homed; sent[1] = (int)pos_X; sent[2] = (int)pos_Y; sent[3] = (int)realSpeedXY_mm[0]; sent[4] = (int)realSpeedXY_mm[1];
    Serial.println(String(sent[0]) + ";" + String(sent[1]) + "," + String(sent[2]) + ";" + String(sent[3]) + "," + String(sent[4]));
  }
}

ISR(TIMER4_COMPB_vect)  {  
  checkGoal();
  static int i = 0;
  if (++i >= RASPBERRY_DATA_LAG) {
    sendDataToRaspberry(false);
    i = 0;
  }
  TCNT4=0;
}
