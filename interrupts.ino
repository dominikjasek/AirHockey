void checkGoal()  {
  static unsigned long lastGoalTimestamp = 0;
  //Serial.println("checking goal");
  if (millis()-lastGoalTimestamp > GOAL_MIN_DELAY)  {
    if (digitalRead(GOAL_ROBOT) == LOW) {
      int i = 0;
      while (digitalRead(GOAL_ROBOT) == LOW)  {
        if (++i == GOAL_SUM_AMOUNT)  {
          Serial.println("gr");
          lastGoalTimestamp = millis(); 
          return;
        }
      }
    }
    else if (digitalRead(GOAL_HUMAN) == LOW) {
      int i = 0;
      while (digitalRead(GOAL_HUMAN) == LOW)  {
        if (++i == GOAL_SUM_AMOUNT)  {
          Serial.println("gh");
          lastGoalTimestamp = millis(); 
          return;
        }
      }
    }
  }
}

void errorTrigger()  {
  error = true;
  error_printed = false;
  if (!homing_state)  {
    homed = false;
  }
  //Serial.println("ERROR TRIGGER");
  //print_pos();
  setZeroSpeeds();
  detachInterrupts();
}

void checkDriverError() {  
  // check if error has occured
  if ((!digitalRead(DRIVER_FLT_0)) || (!digitalRead(DRIVER_FLT_1))) {
    //Serial.println("going to check drivers");
    if (!error_drivers) {
      int i = 0;
      while (!(digitalRead(DRIVER_FLT_0)) || !(digitalRead(DRIVER_FLT_1))) {  // && !error_drivers
        if (++i >= INDUCTION_DRIVER_SWITCH)  {
          error_drivers = true;
          errorTrigger();
          //Serial.println("Driver error...");
          return;
        }
      }
    }
  }
  // check if error has been dismissed
  else {
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
    /*if (changing_dir[1])  {
      Serial.println("motor 1 changing direction iterrupted by doing step");
    }*/
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
    "nop" "\n\t"
    "nop");   // Wait for step pulse
    //CLR(PORTC, PUL2); CLR(x,y) (x&=(~(1<<y))) 
    PORTC&=(~(1<<PUL2));
    //Serial.println("motor 1 step");
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
    /*if (changing_dir[0])  {
      Serial.println("motor 0 changing direction iterrupted by doing step");
    }*/
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
    "nop" "\n\t"
    "nop");  // Wait for step pulse
    //CLR(PORTD, PUL1); 
    //Serial.println("motor 0 step");
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

void sendDataToRaspberry(bool print_home_status)  { //Timer for sending serial data
  static int sent[5] = {0,0,0,0,-1};
  if (error && !error_printed && !homing_state)  {
    if (error_drivers) {
      Serial.println("e2");
    }
    else {
      Serial.println("e1");
    }
    error_printed = true;
  }
  if ((int)pos_X != sent[0] || (int)pos_Y != sent[1] || (int)realSpeedXY_mm[0] != sent[2] || (int)realSpeedXY_mm[1] != sent[3] || print_home_status) {
    sent[0] = (int)pos_X; sent[1] = (int)pos_Y; sent[2] = (int)realSpeedXY_mm[0]; sent[3] = (int)realSpeedXY_mm[1];
    if (print_home_status)  {
      sent[4] = (int)homed; 
      Serial.println(String(sent[0]) + "," + String(sent[1]) + ";" + String(sent[2]) + "," + String(sent[3]) + ";" + String(sent[4]));
    }
    else  {
      Serial.println(String(sent[0]) + "," + String(sent[1]) + ";" + String(sent[2]) + "," + String(sent[3]));
    }
  }
}

ISR(TIMER4_COMPB_vect)  { 
  interrupts(); 
  checkGoal();
  static int i = 0;
  if (++i >= RASPBERRY_DATA_LAG) {
    if (homing_state) {
      sendDataToRaspberry(true);
    }
    else  {
      sendDataToRaspberry(false);
    }
    i = 0;
  }
  TCNT4=0;
}
