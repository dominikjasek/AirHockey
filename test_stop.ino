const int HISTOGRAM_LENGTH = 512;

void recordHistogram()  {
  static uint16_t histogram[HISTOGRAM_LENGTH];
  static uint32_t last_time;
  uint32_t now = micros();
  uint32_t dt = now - last_time;
  uint32_t bin = dt / 4;  // resolution of micros() is 4 us
  if (bin >= HISTOGRAM_LENGTH)
      bin = HISTOGRAM_LENGTH - 1;
  if (++histogram[bin] == UINT16_MAX) {
      print_histogram(histogram);
      exit(0);
  }
  last_time = now;
}

void print_histogram(uint16_t* histogram) {
    Serial.println(F("t (us)  count"));
    Serial.println(F("-------------"));
    for (int i = 0; i < HISTOGRAM_LENGTH; i++) {
        if (histogram[i] == 0) continue;  // skip zeros
        Serial.print(i * 4);
        Serial.print('\t');
        Serial.println(histogram[i]);
    }
    Serial.println(F("-------------"));
    Serial.flush();
}


void test_stop(double accel, double max_speed)  {
  if (!error) {
    Serial.println("Starting test stop");
    positionControl=true;
    double start_X = 80; double start_Y = 0;
    setAccel(accel);
    setMaximalSpeed(1000); 
    setDesiredPosition(80,0);
    while(!positionReached) {
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
      checkDriverError();
      if (!error) {
        updateRealSpeeds();
      }   
    }
    positionControl = false;
    setMaximalSpeed(max_speed); 
  
    Serial.println("moving forward");
    
    setNewDesiredSpeedsXY(max_speed, 0);
    double stop_X = 300;
    double speed_on_line;
    while(pos_X < stop_X) {
      loop();
    }
    
    speed_on_line = realSpeed[0];
    setNewDesiredSpeedsXY(0,0);
  
    for(int i = 0; i<8000; i++) {
      evaluatePos(pos_stepper[0], pos_stepper[1], pos_X, pos_Y);
      checkDriverError();
      if (!error) {
        updateRealSpeeds();
      }   
    }
  
    double diff = pos_X - stop_X;
    Serial.println("Reached Speed = " + String(speed_on_line) + ". Braking distance for a = " + String(accel) + ", v = " + String(max_speed) + " is " + String(diff) + "mm.");
  }
  else
    Serial.println("Cant do test, because there has occured an error!");
  
}
