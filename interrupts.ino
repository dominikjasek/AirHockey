void checkGoal()  {
  if (digitalRead(A2) == LOW) {
    Serial.println("Puck on the goal line!");
  }
}

void errorTrigger()  {
  error = true;
  error_printed = false;
  //Serial.println("ERROR");
  //print_pos();
  setZeroSpeeds();
  detachInterrupts();
}

void checkDriverError() {
  //Serial.println(bitRead(DRIVER_FLT_PIN,DRIVER_FLT_REGISTER_NUM));
  int i = 0;
  while ((!bitRead(DRIVER_FLT_0_PIN,DRIVER_FLT_0_REGISTER_NUM) || !bitRead(DRIVER_FLT_1_PIN,DRIVER_FLT_1_REGISTER_NUM)) && !error) {  // && !error_drivers
    if (++i >= INDUCTION_CONSTANT_SWITCH)  {
      error_drivers = true;
      errorTrigger();
      Serial.println("Driver error...");
      return;
    }
  }
  error_drivers = false;
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
  while (!bitRead(SWITCH_SLIDER_2_PIN,SWITCH_SLIDER_2_REGISTER_NUM)) {
    if (++i >= INDUCTION_CONSTANT_SWITCH)  {
      //Serial.println("slider2 switch");
      switch_slider = true;
      errorTrigger();
      break;
    }
  }
}

void checkSwitchMotor()  {
  int i = 0;
  while (!bitRead(SWITCH_MOTOR_PIN,SWITCH_MOTOR_REGISTER_NUM)) {
    if (++i >= INDUCTION_CONSTANT_SWITCH)  {
      //Serial.println("motor switch");
      switch_motor = true;
      errorTrigger();
      break;
    }
  }
}

void checkSwitchOthers()  {
  int i = 0;
  while (!bitRead(SWITCH_OTHERS_PIN,SWITCH_OTHERS_REGISTER_NUM)) {
    if (++i >= INDUCTION_CONSTANT_SWITCH)  {
      //Serial.println("others switch");
      switch_others = true;
      errorTrigger();
      break;
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

void sendDataToRaspberry()  { //Timer for sending serial data
  static int sent[4] = {0,0,0,0};
  interrupts (); //alow other (motor) interrupts
  if (error && !error_printed)  {
    if (error_drivers) {
      Serial.println("e1");
    }
    else {
      Serial.println("e0");
    }
    error_printed = true;
  }
  else if ((int)pos_X != sent[0] || (int)pos_Y != sent[1] || (int)realSpeedXY_mm[0] != sent[2] || (int)realSpeedXY_mm[1] != sent[3]) {
    sent[0] = (int)pos_X; sent[1] = (int)pos_Y; sent[2] = (int)realSpeedXY_mm[0]; sent[3] = (int)realSpeedXY_mm[1];
    Serial.println(String(sent[0]) + "," + String(sent[1]) + ";" + String(sent[2]) + "," + String(sent[3]));
    /*Serial.print(realSpeedXY_mm[0],0);
    Serial.print(",");
    Serial.println(realSpeedXY_mm[1],0);*/
  }
  //TCNT4=0;  
}

ISR(TIMER4_COMPB_vect)  {  
  checkGoal();
  static int i = 0;
  if (i++ == OCR4A_value/OCR4B_value) {
    sendDataToRaspberry();
    i = 0;
  }
  TCNT4=0;  
}


//Timer0 interrupt - check Goal line
/*ISR(TIMER0_COMPA_vect)  { //Timer for sending serial data
  interrupts ();
  if (digitalRead(A2) == LOW) {
    Serial.println("Puck on the goal line!");
  }
  TCNT0 = 0;
}*/
