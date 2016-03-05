#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>

DualVNH5019MotorShield md;
volatile int pulse_prev_m1, pulse_now_m1, m1_avg_rpm = 0, m1_tick = 0, pulse_prev_m2, pulse_now_m2, m2_avg_rpm = 0, m2_tick = 0;
int pinM1 = 3;
int pinM2 = 5;
// Motor M1 variables
double m1currentrpm, m1newrpm, m1encoder, m1currentspeed;
double m1kp, m1ki, m1kd;
volatile double m1e1, m1e2, m1e3;
double m1k1, m1k2, m1k3;
int count_m1 = 0;

// Motor M2 variables
double m2currentrpm, m2newrpm, m2encoder, m2currentspeed;
double m2kp, m2ki, m2kd;
volatile double m2e1, m2e2, m2e3;
double m2k1, m2k2, m2k3;
int count_m2 = 0;

// Sensors variables
float longadc, longin, longcm;
float shortadc, shortcm;
double maxvalue;
int o;
boolean truefalse = false;

// Ideal RPM
volatile double idealrpm1;
volatile double idealrpm2;

// M1 M2 K1 K2 K3
void computeks()
{
  m1k1 = m1kp + m1ki + m1kd;
  m1k2 = - m1kp - 2 * m1kd;
  m1k3 = m1kd;

  m2k1 = m2kp + m2ki + m2kd;
  m2k2 = - m2kp - 2 * m2kd;
  m2k3 = m2kd;
}

// Init values for PID & ideal RPM
void initial()
{
  m1kp = 2.3; //7.5
  m1ki = 0.4;//0.08
  m1kd = 0.3;//0.35

  m2kp = 3.1; //8
  m2ki = 0.5;//0.02
  m2kd = 0.1;

  m1e1 = 0.0;
  m1e2 = 0.0;
  m1e3 = 0.0;

  m2e1 = 0.0;
  m2e2 = 0.0;
  m2e3 = 0.0;

  idealrpm1 = 93.2;
  idealrpm2 = 95;
}

// Set Motor Speed
void motorspeed()
{
  md.setM1Speed(31);
  md.setM2Speed(31);
  delay(50);

  md.setM1Speed(34);
  md.setM2Speed(32);
  delay(150);

  md.setM1Speed(42);
  md.setM2Speed(40);
  delay(200);

  md.setM1Speed(137);
  md.setM2Speed(134);
  delay(400);

  md.setM1Speed(m1currentspeed);
  md.setM2Speed(m2currentspeed);
}

// Conversion RPM to Speed
void m1rpm2speed()
{
  m1currentspeed = (idealrpm1 + 4.1052) / 0.3957;
}

void m2rpm2speed()
{
  m2currentspeed = (idealrpm2 + 8.4265) / 0.4019;
}

// Calculate PID values with errors
void pidcalculator()
{
  m1e3 = m1e2;
  m1e2 = m1e1;

  m2e3 = m2e2;
  m2e2 = m2e1;

  m1e1 = idealrpm1 - m1currentrpm;
  m2e1 = idealrpm2 - m2currentrpm;

  m1currentspeed = m1currentspeed + m1k1 * m1e1 + m1k2 * m1e2 + m1k3 * m1e3;
  m2currentspeed = m2currentspeed + m2k1 * m2e1 + m2k2 * m2e2 + m2k3 * m2e3;

  md.setM1Speed(m1currentspeed);
  md.setM2Speed(m2currentspeed);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  pinMode(pinM1, INPUT);
  pinMode(pinM2, INPUT);

  digitalWrite(pinM1, LOW);
  digitalWrite(pinM2, LOW);

  PCintPort::attachInterrupt(pinM1, getM1Pulse, CHANGE);
  PCintPort::attachInterrupt(pinM2, getM2Pulse, CHANGE);
  straightsetup();

  //sesnor
  maxvalue = 0;
}

void straightsetup()
{
  initial();
  m1rpm2speed();
  m2rpm2speed();
  computeks();
  motorspeed();
}

int abc;
void loop()
{
  //abc = Serial.read();
 // if (abc == 'A')
  //{

    //Serial.println(m1currentrpm);
    gostraight();
//    for(int i=0;i<4;i++){
//    gostraightblock(2);
//    delay(1500);
//    gostraight1block();
//    delay(1500);}
//    turnright();
//    delay(1500);
//    turnright();
//    delay(1500);
//    for(int i=0;i<4;i++){
//    gostraightblock(2);
//    delay(1500);
//    gostraight1block();
//    delay(1500);}
//    turnright();
//    turnright();
//    delay(100000);
    //delay(25);
    //Serial.println(count_m1);
    //Serial.println(count_m2);
    //delay(1500);


    //Serial.println("A");
    //int a = Serial.read();
    // if(a == 'A')
    // {
    //   turnright();
    // }
    //gostraight();
    /*truefalse = shortrange();
      if (truefalse == true)
      {
      md.setBrakes(390, 400);
      //delay(1000);
      //siam(); //siam2 , brake and delay comment out
      delay(10000);
      }*/
    delay(20);
    //longrange();
 // }
}

void gostraight()
{
  //m1encoder = pulseIn(3, HIGH);
  //m2encoder = pulseIn(5, HIGH);

  //m1currentrpm = 60 / (m1encoder * 2 * 562.25 / 1000000);
  //m2currentrpm = 60 / (m2encoder * 2 * 562.25 / 1000000);
      if ( m1currentrpm != 0 ) {
      m1currentrpm = (m1currentrpm + (period_to_rpm(pulse_prev_m1))) / 2;
    }
    else {
      m1currentrpm = period_to_rpm(pulse_prev_m1);
    }
      if ( m2currentrpm != 0 ) {
      m2currentrpm = (m2currentrpm + (period_to_rpm(pulse_prev_m2))) / 2;
    }
    else {
      m2currentrpm = period_to_rpm(pulse_prev_m2);
    }
  pidcalculator();

  //Serial.println(m1currentrpm);


}



void turnright()
{
  count_m1 = 0;
  count_m2 = 0;
  do
  {
    md.setM1Speed(212);
    md.setM2Speed(-220);

  } while (count_m1 <= 372 || count_m2 <= 372);
  md.setBrakes(400, 400);

}

void turnleft()
{
  count_m1 = 0;
  count_m2 = 0;
  do
  {
    md.setM1Speed(-212);
    md.setM2Speed(220);

  } while (count_m1 <= 392 || count_m2 <= 392);
  md.setBrakes(400, 400);

}

void gostraightblock(int x)
{
  count_m1 = 0;
  count_m2 = 0;
  motorspeed2();
  do
  {
    gostraight();
    delay(20);
  } while (count_m1 <= (303 * x) && count_m2 <= (303 * x));
  md.setBrakes(385, 400);

}

void getM1Pulse() {
  if ( digitalRead(pinM1) == HIGH) {
    pulse_prev_m1 = micros();
    count_m1++;
  }
  else {
    pulse_now_m1 = micros();
    pulse_prev_m1 = pulse_now_m1 - pulse_prev_m1;
    
  }
}


void getM2Pulse() {
  if ( digitalRead(pinM2) == HIGH) {
    pulse_prev_m2 = micros();
    count_m2++;
  }
  else {
    pulse_now_m2 = micros();
    pulse_prev_m2 = pulse_now_m2 - pulse_prev_m2;

  }
}

double period_to_rpm(int time)
{
  double n_rpm = 53357.048 / time;
  return n_rpm;
}

void motorspeed2()
{
  md.setM1Speed(31);
  md.setM2Speed(31);
  delay(50);

  md.setM1Speed(35);
  md.setM2Speed(32);
  delay(150);

  md.setM1Speed(43);
  md.setM2Speed(40);
  delay(200);

  md.setM1Speed(136);
  md.setM2Speed(134);
  delay(200);

  md.setM1Speed(185);
  md.setM2Speed(183);
  delay(200);

  md.setM1Speed(216);
  md.setM2Speed(213);
  delay(200);

  md.setM1Speed(m1currentspeed);
  md.setM2Speed(m2currentspeed);
}

void gostraight1block()
{
  md.setM1Speed(36);
  md.setM2Speed(31);
  delay(50);

  md.setM1Speed(38);
  md.setM2Speed(31);
  delay(150);

  md.setM1Speed(46);
  md.setM2Speed(39);
  delay(200);

  md.setM1Speed(134);
  md.setM2Speed(134);
  delay(200);

  md.setM1Speed(185);
  md.setM2Speed(184);
  delay(200);

  md.setM1Speed(215);
  md.setM2Speed(214);
  delay(200);

  do
  {
    md.setM1Speed(251);
    md.setM2Speed(253);
  } while (count_m1 <= (301) && count_m2 <= (301));

  md.setBrakes(400, 395);
  count_m1 = 0;
  count_m2 = 0;
}

void turn720right()
{
  count_m1 = 0;
  count_m2 = 0;
  do
  {
    md.setM1Speed(212);
    md.setM2Speed(-220);

  } while (count_m1 <= 3080 || count_m2 <= 3080);
  md.setBrakes(400, 400);
}

void turn720left()
{
  count_m1 = 0;
  count_m2 = 0;
  do
  {
    md.setM1Speed(-212);
    md.setM2Speed(220);

  } while (count_m1 <= 3230 || count_m2 <= 3240);
  md.setBrakes(400, 400);
}

// Reading Long Range Sensor Distance
void longrange()
{
  longadc = analogRead(1);
  longin = 6202.3 * pow(longadc, -1.056); // formula to convert ADC reading to inches
  longcm = longin * 2.54; // inches to cm

  Serial.print(" Long ADC   ");
  Serial.println(longadc);

  Serial.print("cm   ");
  Serial.println(longcm);
  delay(100);
}

// Reading Short Range Sensor Distance (accurate until 40cm)
boolean shortrange()
{
  shortadc = analogRead(0);
  shortcm = (6762 / (shortadc - 9) ) - 4 - 10;
  /*
    if (shortcm > maxvalue)
    {
      maxvalue = shortcm;
    }
    if (shortcm > int(maxvalue))
    {

      Serial.print("Short ADC   ");
      Serial.println(shortadc);

      Serial.print("cm   ");
      if (shortcm <= 31)
      {
        Serial.println(int(shortcm) - 10);
        shortcm = shortcm - 10;
      }
      delay(100);
    }*/
  if (shortcm < 7) //drift 30
  {
    return true;
  }
  else
  {
    return false;
  }


}


void siam()
{
  turnright();
  count_m1 = 0;
  count_m2 = 0;
  delay(1500);
  gostraightblock(2);
  delay(1500);
  turnleft();
  count_m1 = 0;
  count_m2 = 0;
  delay(1500);
  gostraightblock(6);
  delay(1500);
  turnleft();
  count_m1 = 0;
  count_m2 = 0;
  delay(1500);
  gostraightblock(2);
  delay(1500);
  turnright();
  count_m1 = 0;
  count_m2 = 0;
  delay(1500);
  gostraightblock(2);
  delay(1500);
}

void siamdiu()
{
  md.setM1Speed(270);
  md.setM2Speed(150);
  delay(1400);
  md.setM1Speed(150);
  md.setM2Speed(270);
  delay(2800);
  md.setM1Speed(270);
  md.setM2Speed(150);
  delay(1450);
  md.setM1Speed(260);
  md.setM2Speed(270);
}

