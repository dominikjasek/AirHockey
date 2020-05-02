// Reading variables
char home_keyword[]  = "home";
char demo_keyword[]  = "demo";
char test_keyword[]  = "test";
char cyclic_test_keyword[] = "cyclic_test";
char position_keyword[]  = "p";
char velocity_keyword[]  = "v";
char motors_keyword[]  = "m";
char preg_keyword[]  = "kpgain";
char default_keyword[]  = "default";
char set_acceleration_keyword[]  = "setaccel";
char set_decelgain_keyword[]  = "setdecelgain";
char set_maxspeed_keyword[]  = "setmaxspeed";
char preventwallhit_keyword[]  = "preventwallhit";
char led_keyword[] = "leds";
char fan[] = "fans";
char solenoid[] = "solenoid";
int control_mode = 3;
char recievedChar;
char * strtokIndx;
char buf[20];

//declare default params (arduino IDE doesn't support this in a classic way)
void test_single(float accel, unsigned int max_speed, double rad = 0, double start_Y = 0);

void checkSerialInput() {
  strcpy(buf,"");
  while (Serial.available())  {
    readline(Serial.read(), buf, 80);
  }
  if ((buf[0] != NULL)) {
      // credit: https://forum.arduino.cc/index.php?topic=288234.0
      strtokIndx  = strtok(buf,",");  //parse buf into part ending with ","
      
      if (strcmp(strtokIndx,home_keyword) == 0) {
        //Serial.println("going to homing");
        homing();        
      }
      else if (strcmp(strtokIndx,position_keyword) == 0) {  // set new desired position
         
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         float value0 = atof(strtokIndx);  //convert string to integer
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         float value1 = atof(strtokIndx);
         //Serial.println("set new desired position: " + String(value0) + ", " + String(value1));
         setDesiredPosition(value0,value1);    
      }
      else if (strcmp(strtokIndx,velocity_keyword) == 0) {  // set XY speed
         positionControl = false;
         //Serial.print("set XY speed: ");
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         float value0 = atof(strtokIndx);  //convert string to integer
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         float value1 = atof(strtokIndx);
         //Serial.println("red speed in mm = " + String(value0) + " " + String(value1));
         setDesiredSpeedsXY(value0, value1);       
      }
      else if (strcmp(strtokIndx,motors_keyword) == 0) {  // control motors speed
         positionControl = false;
         //Serial.print("control motors speed: ");
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         float value0 = atof(strtokIndx);  //convert string to integer
         //value0 = mmToSteps(value0);
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         float value1 = atof(strtokIndx);   
         //value1 = mmToSteps(value1);
         //Serial.println("Setting motors speeds = " + String(value0) + " " + String(value1));   
         setDesiredSpeedsMotors(value0, value1);       
      }
      else if (strcmp(strtokIndx,demo_keyword) == 0) {
        demo();        
      }
      else if (strcmp(strtokIndx,set_acceleration_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        float acc = atof(strtokIndx);  //convert string to integer
        setAccel(acc);  
      }
      else if (strcmp(strtokIndx,set_decelgain_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        const int d = atoi(strtokIndx);  //convert string to integer
        setDecel(d);  
      }
      else if (strcmp(strtokIndx,set_maxspeed_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        float value0 = atof(strtokIndx);  //convert string to integer
        setMaximalSpeed(value0);  
      }
      else if (strcmp(strtokIndx,cyclic_test_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        float acceleration = atof(strtokIndx);  //convert string to integer
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        int rad_distrib_num = atoi(strtokIndx);  //convert string to integer
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        int speed_add = atoi(strtokIndx);  //convert string to integer
        test_cyclic(acceleration, rad_distrib_num, speed_add);   
      }
      else if (strcmp(strtokIndx,test_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        float acc = atof(strtokIndx);  //convert string to integer
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        unsigned int max_speed = atoi(strtokIndx); 
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        double angle = atof(strtokIndx); 
        //Serial.println("Angle = " + String(angle));
        delay(5);
        test_single(acc, max_speed, angle);  
      }
      else if (strcmp(strtokIndx,preg_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        unsigned int value0 = atoi(strtokIndx);  //convert string to integer
        setKpGain(value0);  
      }
      else if (strcmp(strtokIndx,default_keyword) == 0) {
        setDefaultParams();
      }
      else if (strcmp(strtokIndx,preventwallhit_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        int _value = atoi(strtokIndx);  //convert string to integer
        preventWallHit = (_value == 1)? (true) : (false);
      }
      else if (strcmp(strtokIndx,solenoid) == 0) {
        pushSolenoid();
      }
      else if (strcmp(strtokIndx,fan) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        int _value = atoi(strtokIndx);  //convert string to integer
        manipulateFan(_value);
      }
      else if (strcmp(strtokIndx,led_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        int _value = atoi(strtokIndx);  //convert string to integer
        led(_value);
      }
      else {
        Serial.println("Bullshit: " + String(strtokIndx));
      }
      //print_pos();
   }
}

int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;
    if (readch > 0) {
        switch (readch) {
           default:
              if (pos < len-1) {
                  buffer[pos++] = readch;
                  buffer[pos] = 0;
              }
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
           
        }
    }
    return 0;
}

void pushSolenoid() {
  static unsigned long timeStamp = 0;
  if ((millis() - timeStamp) > SOLENOID_MIN_DELAY) {
    bool turn_on_fans = false;
    if (fan_state)  {
      manipulateFan(0);
      delay(5);
      turn_on_fans = true;
    }    
    digitalWrite(SOLENOID_PIN, LOW);
    delay(50);
    digitalWrite(SOLENOID_PIN, HIGH);
    if (turn_on_fans)  {
      manipulateFan(1);
      delay(5);
    }
    timeStamp = millis();
  }

}

void manipulateFan(int state) {
  fan_state = state;
  if (state)  {
    digitalWrite(FANS_PIN, LOW);
  }
  else  {
    digitalWrite(FANS_PIN, HIGH);
  }
}

void led(int _value)  {
  if (_value >= 0 && _value <= 255) {
    analogWrite(LED_STRIP,_value);
  }
  else {
    Serial.println("You set wrong led value");
  }
}

void setAccel(float _accel_per1sec) {
  //_accel_per1sec/=2.0;  //_accel_per1sec is acceleration for motor, not axis!!!!
  if (_accel_per1sec > 0) {
      ACCEL_PER1SEC = _accel_per1sec;
      //Serial.println("ACCEL_PER1SEC = " + String(ACCEL_PER1SEC));
      _accel_per1sec*=0.5;  //h-bot construction;
      ACCEL = mmToSteps((float)((_accel_per1sec*CYCLE_DURATION)/1000000.0));
      //Serial.print("Setting accelertion = ");
      //Serial.println(ACCEL, 4);
      pickCoefficients();
  }
  else 
    Serial.println("Acceleration must be greater than 0 and lower than " + String(MAX_ALLOWED_ACCEL_PER1SEC));
}

void setDecel(int _DECEL_GAIN) {
  DECEL_GAIN = _DECEL_GAIN;
  DECEL = DECEL_GAIN * ACCEL;
  Serial.println("DECEL set to " + String(DECEL));
}

void setMaximalSpeed(float _maxspeed) {
  MM_SPEED = _maxspeed;
  _maxspeed = mmToSteps(_maxspeed);
  //Serial.println(_maxspeed);
  _maxspeed/=2; //max speed , XY coordinates is 2 times larger than max speed of motor
  if (_maxspeed > 0) {
    if (_maxspeed > MAX_MOTOR_ALLOWED_SPEED) {
      MAX_MOTOR_SPEED = MAX_MOTOR_ALLOWED_SPEED;
    }
    else {
      MAX_MOTOR_SPEED = _maxspeed;
    }
    //Serial.print("Setting maximal speed = ");
    //Serial.println(MAX_MOTOR_SPEED);
  }
  else 
    Serial.println("Max speed must be greater than 0 and lower than " + String(MAX_MOTOR_ALLOWED_SPEED));
}

void setKpGain(unsigned int _Kp) {
  if (_Kp > 0 && _Kp < MAX_KPGAIN)
    Kp = _Kp;
}

/*------------------------------------------------------------------------------------------------*/

void print_pos()  {
  Serial.print("Position X,Y: ");
  Serial.println(String(pos_X) + " " + String(pos_Y));
}

void print_desired_pos()  {
  Serial.print("Desired position X,Y: ");
  Serial.println(String(desiredPosition[0]) + " " + String(desiredPosition[1]));
}

void print_steps()  {
  Serial.print("Steppers steps 1,2: ");
  Serial.println(String(pos_stepper[0]) + " " + String(pos_stepper[1]));
}

void print_real_speeds()  {
  Serial.print("Real speeds: ");
  Serial.println(String(realSpeed[0]) + " " + String(realSpeed[1]));
}

void print_real_speedsXY()  {
  Serial.print("Real speeds XY: ");
  Serial.println(String(realSpeedXY_mm[0]) + " " + String(realSpeedXY_mm[1]));
}

void print_desired_speeds()  {
  Serial.print("Desired speeds: ");
  Serial.println(String(desiredSpeed[0]) + " " + String(desiredSpeed[1]));
}

void print_error()  {
  Serial.print("Error=");
  Serial.println(error);
}
