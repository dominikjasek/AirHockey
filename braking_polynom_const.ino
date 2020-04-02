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
        Serial.println("Coefficient for a = 30000 mms^-2 were chosen.");
        offset = 155;  //mm
        p10 = 0.07017;
        p20 = 0.00006;
        p11 = -0.0003183;
        p30 = -0.00000001859;
        p21 = 0.00000004978;
        p40 = 0.000000000003181;
        p31 = -0.0000000000002423;
        p22 = -0.000000000006265;
        break;
      /*case (30000): //stare jednotky
        offset = 10;
        p10 = 0.0327/0.15;
        p20 = 0.0009032/0.15/0.15;
        p11 = -0.002755/0.15/0.15;
        p30 = -6.71E-07/0.15/0.15/0.15;
        p21 = 1.18E-06/0.15/0.15/0.15;
        p40 = 2.76E-10/0.15/0.15/0.15/0.15;
        p31 = 1.27E-11/0.15/0.15/0.15/0.15;
        p22 = -4.63E-10/0.15/0.15/0.15/0.15;
        break;*/
      case (15000):
        offset = 10;
        p10 = 0.001426;
        p20 = 0.0003261;
        p11 = -0.000915;
        p30 = -1.37E-07;
        p21 = 2.25E-07;
        p40 = 3.18E-11;
        p31 = 2.65E-12;
        p22 = -5.22E-11;
        break;
      case (50000):
        offset = 10;      
        p10 = -0.01122;
        p20 = 0.0001991;
        p11 = -0.0005213;
        p30 = -0.00000006385;
        p21 = 0.00000009921;
        p40 = 0.00000000001162;
        p31 = 0.000000000001237;
        p22 = -0.00000000001821;
        break;
      case (70000):
        offset = 80;      
        p10 = -0.007325;
        p20 = 0.0001443;
        p11 = -0.0003812;
        p30 = -3.95E-08;
        p21 = 6.20E-08;
        p40 = 6.07E-12;
        p31 = 7.60E-13;
        p22 = -9.92E-12;
        break;
      case (90000):
        offset = 80;      
        p10 = 0.004689;
        p20 = 0.0001103;
        p11 = -0.0003114;
        p30 = -2.75E-08;
        p21 = 4.53E-08;
        p40 = 3.76E-12;
        p31 = 3.47E-13;
        p22 = -6.22E-12;
        break;
    }
}

// a = 50000
double criticalDist(double realSpeed0, double realSpeed1)  {
  double x = abs(realSpeed0);
  double y = abs(realSpeed1);
  double xy = x*y;
  double x2 = x*x;
  double y2 = y*y;
  return offset + p10*(x+y)+p11*xy + p20*(x*x + y*y) + xy*p21*(x+y) + p30*(x2*x + y2*y) + p40*(x2*x2+y2*y2) + p31*xy*(x2+y2) + p22*x2*y2;
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
//  return p0 + p10*(x+y)+p11*xy + p20*(x*x + y*y) + xy*p21*(x+y) + p30*(x2*x + y2*y) + p40*(x2*x2+y2*y2) + p31*xy*(x2+y2) + p22*x2*y2;
//}
