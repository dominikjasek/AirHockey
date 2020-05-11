/*int offset;
double p10;
double p20;
double p11;
double p30;
double p21;
double p40;
double p31;
double p22;

void pickCoefficients(unsigned int decel)  {
  Serial.println("Picking braking coefficients for decelper1sec = " + String(decel));
  switch (decel)
    {
      case (30000):
        Serial.println("Coefficient for a = 30000 mms^-2 were chosen.");
        offset = 320;  //mm
        p10 = 0.07017;
        p20 = 0.00006;
        p11 = -0.0003183;
        p30 = -0.00000001859;
        p21 = 0.00000004978;
        p40 = 0.000000000003181;
        p31 = -0.0000000000002423;
        p22 = -0.000000000006265;
        break;      
      case (10000):
        Serial.println("Coefficient for a = 10000 mms^-2 were chosen.");
        offset = 260;
        p10 = -0.003314;
        p20 = 0.0002464;
        p11 = -0.0006785;
        p30 = -0.0000001052;
        p21 = 0.0000001761;
        p40 = 0.00000000002617;
        p31 = 0.000000000002037;
        p22 = -0.00000000004315;
        break;
      case (20000):
        Serial.println("Coefficient for a = 20000 mms^-2 were chosen.");
        offset = 300;      
        p10 = 0.01793;
        p20 = 0.0001257;
        p11 = -0.0003948;
        p30 = -0.00000003698;
        p21 = 0.00000006987;
        p40 = 0.000000000006532;
        p31 = 0.0000000000005114;
        p22 = -0.00000000001155;
        break;
      case (5000):
        Serial.println("Coefficient for a = 5000 mms^-2 were chosen.");
        offset = 300;      
        p10 = 0.439;
        p20 = -0.00005849;
        p11 = -0.002195;
        p30 = -0.00000008309;
        p21 = 0.0000008639;
        p40 = 0.00000000007719;
        p31 = -0.00000000004357;
        p22 = -0.0000000002227;
        break;
      case (15000):
        Serial.println("Coefficient for a = 15000 mms^-2 were chosen.");
        offset = 300;      
        p10 = 0.128;
        p20 = 0.00006056;
        p11 = -0.0005827;
        p30 = -0.00000003388;
        p21 = 0.0000001353;
        p40 = 0.00000000001026;
        p31 = -0.000000000002314;
        p22 = -0.00000000002341;
        break;
      case (25000):
        Serial.println("Coefficient for a = 25000 mms^-2 were chosen.");
        offset = 400;      
        p10 = 0.01676;
        p20 = 0.0001033;
        p11 = -0.000324;
        p30 = -0.00000002777;
        p21 = 0.00000005219;
        p40 = 0.000000000004367;
        p31 = 0.0000000000002409;
        p22 = -0.000000000007664;
        break;
    }
}

double criticalDist(double realSpeed0, double realSpeed1)  {
  double x = abs(realSpeed0);
  double y = abs(realSpeed1);
  double xy = x*y;
  double x2 = x*x;
  double y2 = y*y;
  return offset + p10*(x+y)+p11*xy + p20*(x*x + y*y) + xy*p21*(x+y) + p30*(x2*x + y2*y) + p40*(x2*x2+y2*y2) + p31*xy*(x2+y2) + p22*x2*y2;
}
*/

int cum_sum(const float _nr_cycles) {
  int _nr_cycles_rounded = ceil(_nr_cycles);
  int sum = ((_nr_cycles_rounded + 1)*_nr_cycles_rounded)/2;  // Arithmetic progression formula
  return sum;
}

int stepsToBeDone(float _realSpeed, float _decel) {
  if (_decel == 0)
    return 0;
    
  float nr_cycles = abs(_realSpeed/_decel); //number of cycles needed to stop, should be same as realSpeed[1]/desiredBrakingAccel[1];
  //Serial.println("_decel = "+ String(_decel));
  //Serial.println("nr_cycles = " + String(nr_cycles));
  int steps = (_decel*CYCLE_DURATION/1000000)*cum_sum(nr_cycles);
  //Serial.println("steps = " + String(steps));
  return steps;
}

double brakingDist() {
  // input - realSpeed[0], realSpeed[1]
  // output - braking distance
  
  scaleToMaxMotorValues(realSpeed[0], realSpeed[1], DECEL, DECEL, desiredBrakingAccel[0], desiredBrakingAccel[1]);

  int steps0 = stepsToBeDone(realSpeed[0], desiredBrakingAccel[0]);
  int steps1 = stepsToBeDone(realSpeed[1], desiredBrakingAccel[1]);

  evaluatePos(pos_stepper[0] + steps0,pos_stepper[1] + steps1, pos_braking_X,pos_braking_Y);
  return BRAKING_OFFSET + distSqr(pos_X, pos_Y, pos_braking_X,pos_braking_Y); 
}


double minDistToWall()  {
  double x_hit; double y_hit; double tgAlpha;
  tgAlpha = realSpeedXY_mm[1]/realSpeedXY_mm[0];
  if (realSpeedXY_mm[0] >= 0) {    
    double x0 = BARRIER_X_MAX - pos_X;
    double y0 = x0*tgAlpha;
    double y0_tot = y0 + pos_Y;
    if (isnan(y0_tot)) {
      return 80000;
    }
    else if (y0_tot > BARRIER_Y_MAX) { //hitting upper X line
      //Serial.println("upper");
      y_hit = BARRIER_Y_MAX;
      x_hit = BARRIER_X_MAX - (y0_tot - BARRIER_Y_MAX)/tgAlpha;
    }
    else if (y0_tot < BARRIER_Y_MIN)  { //hitting lower X line
      //Serial.println("lower");
      y_hit = BARRIER_Y_MIN;
      x_hit = BARRIER_X_MAX - (y0_tot - BARRIER_Y_MIN)/tgAlpha;
    }
    else  { //hitting X_MAX line
      //Serial.println("back");
      x_hit = BARRIER_X_MAX;
      y_hit = y0_tot;
    }
  }
  else  {
    double x0 = pos_X - BARRIER_X_MIN;
    double y0 = -x0*tgAlpha;
    double y0_tot = y0 + pos_Y;
    //Serial.println("y0_tot = " + String(y0_tot));
    //Serial.println("------------");
    if (isnan(y0_tot)) {
      return 80000;
    }
    else if (y0_tot > BARRIER_Y_MAX) { //hitting upper X line
      //Serial.println("upper");
      y_hit = BARRIER_Y_MAX;
      x_hit = BARRIER_X_MIN - (y0_tot - BARRIER_Y_MAX)/tgAlpha;
    }
    else if (y0_tot < BARRIER_Y_MIN)  { //hitting lower X line
      //Serial.println("lower");
      y_hit = BARRIER_Y_MIN;
      x_hit = BARRIER_X_MIN - (y0_tot - BARRIER_Y_MIN)/tgAlpha;
    }
    else  { //hitting X_MIN line
      //Serial.println("front");
      x_hit = BARRIER_X_MIN;
      y_hit = y0_tot;
    }
  }
  
  if (isnan(x_hit)) 
    x_hit = pos_X;

  
  double distToWall = distSqr(pos_X, pos_Y, x_hit, y_hit);
  if (isnan(distToWall) || distToWall < 0) 
    distToWall = 80000;
    
  //Serial.println("Intersect point = [" + String(x_hit) + ", " + String(y_hit) + "]");
  //Serial.println("Dist to wall = " + String(distToWall));
  
  return distToWall;
}

double distSqr(double x1, double y1, double x2, double y2) {
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}

void preventWallCollision() {
  double braking_dist = brakingDist();
  double dist = minDistToWall();
  if (braking_dist > dist) {
    caution_braking = true;
    CYCLE_DURATION = FAST_CYCLE_DURATION;
    Serial.println("CAUTION MODE ACTIVATED");
    setDesiredSpeedsXY(0,0);
    //positionControl = false;
  }
}
