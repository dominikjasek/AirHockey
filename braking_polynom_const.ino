int offset;
double p10;
double p20;
double p11;
double p30;
double p21;
double p40;
double p31;
double p22;

void pickCoefficients()  {
  switch ((int)ACCEL_PER1SEC)
    {
      case (30000):
        //Serial.println("Coefficient for a = 30000 mms^-2 were chosen.");
        offset = 300;  //mm
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
        //Serial.println("Coefficient for a = 10000 mms^-2 were chosen.");
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
        //Serial.println("Coefficient for a = 20000 mms^-2 were chosen.");
        offset = 260;      
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
        offset = 290;      
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
        //Serial.println("Coefficient for a = 25000 mms^-2 were chosen.");
        offset = 260;      
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

/*int offset;
double p00;
double p10;
double p20;
double p11;

void pickCoefficients()  {
  switch ((int)ACCEL_PER1SEC)
    {      
      case (25000):
        Serial.println("Coefficient for a = 25000 mms^-2 were chosen.");
        offset = 5;   //mm   
        p00 = 1.584;
        p10 = 0.001094;
        p20 = 0.000001527;
        p11 = -0.0000009317;
        break;
    }
}

double criticalDist(double realSpeed0, double realSpeed1)  {
  double x = abs(realSpeed0);
  double y = abs(realSpeed1);
  return offset + p00 + p10*(x+y)+p11*x*y + p20*(x*x + y*y);
}

//// a = 10000
//double allowedDist10000()  {
//  const int offset = 180;
//  const double p0 = 0 + offset;
//  const double p10 = 0.03115;
//  const double p20 = 0.0008602;
//  const double p11 = -0.002624;
//  const double p30 = -0.0000006392;
//  const double p21 = 0.000001125;
//  const double p40 = 0.0000000002624;
//  const double p31 = 0.00000000001211;
//  const double p22 = -0.0000000004413;
//  double x = abs(realSpeed[0]);
//  double y = abs(realSpeed[1]);
//  double xy = x*y;
//  double x2 = x*x;
//  double y2 = y*y;
//  return p0 + p10*(x+y)+p11*xy + p20*(x*x + y*y) + xy*p21*(x+y) + p30*(x2*x + y2*y) + p40*(x2*x2+y2*y2) + p31*xy*(x2+y2) + p22*x2*y2;*/
//}
