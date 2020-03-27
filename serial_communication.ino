// Reading variables
char home_keyword[]  = "home";
char demo_keyword[]  = "demo";
char demo2_keyword[]  = "demo2";
char test_keyword[]  = "test";
char position_keyword[]  = "p";
char velocity_keyword[]  = "v";
char motors_keyword[]  = "m";
char preg_keyword[]  = "kpgain";
char set_acceleration_keyword[]  = "setaccel";
char set_maxspeed_keyword[]  = "setmaxspeed";
int control_mode = 3;
char recievedChar;
char * strtokIndx;
char buf[80];

void checkSerialInput() {
  if (readline(Serial.read(), buf, 80) > 0) {
      // credit: https://forum.arduino.cc/index.php?topic=288234.0
      strtokIndx  = strtok(buf,",");  //parse buf into part ending with ","
      if (strcmp(strtokIndx,home_keyword) == 0) {
        homing(homing_state,error, positionControl);        
      }
      
//      else if (strcmp(strtokIndx,demo2_keyword) == 0) {
//        demo2();        
//      }
      else if (strcmp(strtokIndx,position_keyword) == 0) {  // set new desired position
         positionControl = true;
         //Serial.println("set new desired position");
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         double value0 = atoi(strtokIndx);  //convert string to integer
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         double value1 = atoi(strtokIndx);
         setDesiredPosition(value0,value1);    
      }
      else if (strcmp(strtokIndx,velocity_keyword) == 0) {  // set XY speed
         positionControl = false;
         //Serial.print("set XY speed: ");
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         double value0 = atoi(strtokIndx);  //convert string to integer
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         double value1 = atoi(strtokIndx);
         Serial.println(String(value0) + " " + String(value1));
         setNewDesiredSpeedsXY(value0, value1);       
      }
      else if (strcmp(strtokIndx,motors_keyword) == 0) {  // control motors speed
         positionControl = false;
         //Serial.print("control motors speed: ");
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         double value0 = atoi(strtokIndx);  //convert string to integer
         strtokIndx = strtok(NULL, ","); //parse same strtokIndx
         double value1 = atoi(strtokIndx); 
         //Serial.println(String(value0) + " " + String(value1));
         setNewDesiredSpeedsMotors(value0, value1);       
      }
      else if (strcmp(strtokIndx,demo_keyword) == 0) {
        demo();        
      }
      else if (strcmp(strtokIndx,set_acceleration_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        double value0 = atoi(strtokIndx);  //convert string to integer
        setAccel(value0);  
      }
      else if (strcmp(strtokIndx,set_maxspeed_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        double value0 = atoi(strtokIndx);  //convert string to integer
        setMaximalSpeed(value0);  
      }
      else if (strcmp(strtokIndx,test_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        double value0 = atoi(strtokIndx);  //convert string to integer
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        double value1 = atoi(strtokIndx); 
        delay(10);
        test_stop(value0, value1);  
      }
      else if (strcmp(strtokIndx,preg_keyword) == 0) {
        strtokIndx = strtok(NULL, ","); //parse same strtokIndx
        double value0 = atoi(strtokIndx);  //convert string to integer
        setKpGain(value0);  
      }
      print_pos();
   }
   
}

int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;
    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

void setAccel(double _accel) {
  if (_accel > 0 && _accel <= MAX_ALLOWED_ACCEL) {
    MAX_ACCEL = _accel;
    Serial.print("Setting accelertion = ");
    Serial.println(MAX_ACCEL);
  }
  else 
    Serial.println("Acceleration must be greater than 0!");
}

void setMaximalSpeed(double _maxspeed) {
  if (_maxspeed > 0 && _maxspeed <= MAX_ALLOWED_SPEED) {
    MAX_SPEED = _maxspeed;
    Serial.print("Setting maximal speed = ");
    Serial.println(MAX_SPEED);
  }
  else 
    Serial.println("Max speed must be greater than 0!");
}

void setKpGain(double _Kp) {
  if (_Kp > 0 && _Kp < MAX_KPGAIN)
    Kp = _Kp;
}
