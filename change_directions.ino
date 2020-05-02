void changeDirection0()  {
  //noInterrupts();
  //Serial.println("changing direction of motor 0");
  //Serial.println(micros());
  //changing_dir[0] = true;
  //TCNT3 = 0;
  PORTD ^= (1 << DIR1);
  delayMicroseconds(5); 
  //Serial.println(micros());
  //changing_dir[0] = false;  
  //interrupts();
}

void changeDirection1() {
  //noInterrupts();
  //Serial.println("changing direction of motor 1");
  //changing_dir[1] = true;
  //TCNT1 = 0;
  PORTD ^= (1 << DIR2);
  delayMicroseconds(5);  
  //changing_dir[1] = false;  
  //interrupts();
}
