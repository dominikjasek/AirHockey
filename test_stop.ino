void test_cyclic(float accel, int angle_distribution_num, int add_value_speed)  {
   unsigned int max_speed = 50;
   double add_angle = 1.5708/angle_distribution_num;
   Serial.println("Acceleration; Motor0; Motor1; braking_distance");
   while (!error) {
    double rad = 0;
    for (int i = 0; i < angle_distribution_num; i++)  {
      test_single(accel,max_speed,rad,-230);
      rad += add_angle;
    }    
    max_speed += add_value_speed;
   } 
}

void test_single(float accel, unsigned int max_speed, double rad, double start_Y)  {
  if (!error) {
    homing_state = true;
    setDefaultParams();
    //Serial.println("Starting test stop");
    
    double start_X = 70;
    waitUntilPosReached(start_X,start_Y);
    /*setDesiredPosition(start_X,start_Y);
    while(distSqr(pos_X,pos_Y,start_X,start_Y) > POSITION_ERROR_TOLERANCE) {
      loop(); 
    }*/
    positionControl = false;
    setAccel(accel);
    setMaximalSpeed(max_speed); 
  
    //Serial.println("moving forward");
    setDesiredSpeedsXY(50000*cos(rad), 50000*sin(rad));
    //print_desired_speeds();    
    
    while(abs(realSpeed[0]) < abs(desiredSpeed[0]) || abs(realSpeed[1]) < abs(desiredSpeed[1])) {
      //Serial.println("--------------");
      loop();
      //print_real_speeds();
      //print_desired_speeds();
    }

    int max_test_speed[2] = {realSpeed[0], realSpeed[1]};
    double braking_pos[2] = {pos_X, pos_Y};
    //Serial.println("braking_pos = " + String(braking_pos[0]) + " " + String(braking_pos[1]));
    setDesiredSpeedsXY(0,0);
    //print_real_speedsXY();
    while(abs(realSpeed[0]) > 0.2 || abs(realSpeed[1]) > 0.2) {
      loop();
      //Serial.println("i = " + String(i));
    }  
    
    double braking_distance = distSqr(pos_X, pos_Y, braking_pos[0], braking_pos[1]);
    Serial.println(String(accel) + ";" + String(max_test_speed[0]) + ";" + String(max_test_speed[1]) + ";" + String(braking_distance));
    //print_pos();
    setDefaultParams();
    delay(10);
    homing_state = true;
  }
  else
    Serial.println("Cant do test, because there has occured an error!");
}
