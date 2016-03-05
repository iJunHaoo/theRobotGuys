#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <SharpIR.h>

DualVNH5019MotorShield md;

/***************************** Sensors Variables ***********************************************/
#define sRight A1
#define sLeft A2
#define sBottom A3
#define sMiddle A4

double sensorSR [10];
double sensorSL [10];
double sensorSB [10];
double sensorSM [10];

SharpIR sR (sRight , 1080);
SharpIR sL (sLeft , 1080);
SharpIR sB (sBottom , 1080);
SharpIR sM (sMiddle , 1080);

double leftSensorValue, rightSensorValue;
/**********************************************************************************************/



/*****************************  Motor Variables  ***********************************************/
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

// Ideal RPM
volatile double idealrpm1;
volatile double idealrpm2;
/*********************************************************************************************/



/***************************** User Variable **************************************************/
char userInput;
/**********************************************************************************************/



/***************************** Motors Kp, Ki, Kd ***********************************************/
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
/***********************************************************************************************/



/***************************** Calculate K1, K2, K3 ******************************************/
void computeks()
{
  m1k1 = m1kp + m1ki + m1kd;
  m1k2 = - m1kp - 2 * m1kd;
  m1k3 = m1kd;

  m2k1 = m2kp + m2ki + m2kd;
  m2k2 = - m2kp - 2 * m2kd;
  m2k3 = m2kd;
}
/*********************************************************************************************/



/***************************** M1 RPM - SPEED ***********************************************/
void m1rpm2speed()
{
  m1currentspeed = (idealrpm1 + 4.1052) / 0.3957;
}
/*********************************************************************************************/



/***************************** M2 RPM - SPEED ***********************************************/
void m2rpm2speed()
{
  m2currentspeed = (idealrpm2 + 8.4265) / 0.4019;
}
/*********************************************************************************************/



/***************************** PID Calculator ***********************************************/
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
/*********************************************************************************************/



/***************************** Motor Straight Setup *********************************************/
void straightsetup()
{
  initial();
  m1rpm2speed();
  m2rpm2speed();
  computeks();
}
/***********************************************************************************************/



/************************************* Main Setup **********************************************/
void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();

  pinMode(0, OUTPUT);
  analogWrite(0, 0);
  pinMode(5, OUTPUT);
  analogWrite(5, 0);

  pinMode(pinM1, INPUT);
  pinMode(pinM2, INPUT);

  digitalWrite(pinM1, LOW);
  digitalWrite(pinM2, LOW);

  PCintPort::attachInterrupt(pinM1, getM1Pulse, CHANGE);
  PCintPort::attachInterrupt(pinM2, getM2Pulse, CHANGE);
  straightsetup();
}
/*********************************************************************************************/



/***************************** Main Loop *************************************************/
void loop()
{
  userInput = Serial.read();

  switch (userInput)
  {
    case 'A' :
      Serial.println("B");
      break;

    case 'B' :
      for (int i = 0; i < 10; i++)
      {
        readingSensors();
        Serial.print("Left Sensor Value ");
        Serial.print(leftSensorValue);
        Serial.println("");
        Serial.print("Right Sensor Value ");
        Serial.print(rightSensorValue);
        Serial.println("");
      }
      break;

    case 'w' :
      gostraightblock(1);
      delay(150);
      break;

    case 'e' :
      gostraightblock(2);
      delay(150);
      break;

    case 'r' :
      gostraightblock(3);
      delay(150);
      break;

    case 't' :
      gostraightblock(4);
      delay(150);
      break;

    case 'y' :
      gostraightblock(5);
      delay(150);
      break;

    case 'u' :
      gostraightblock(6);
      delay(150);
      break;

    case 'i' :
      gostraightblock(7);
      delay(150);
      break;

    case 'o' :
      gostraightblock(8);
      delay(150);
      break;

    case 'p' :
      gostraightblock(9);
      delay(150);
      break;

    case 'a' :
      turnleft();
      delay(150);
      break;

    case 's' :
      turnright();
      delay(150);
      turnright();
      delay(150);
      break;

    case 'd' :
      turnright();
      delay(150);
      break;

    case 'z' :
      readingSensors();
      while (leftSensorValue != rightSensorValue)
      {
        autoalign(leftSensorValue, rightSensorValue);
        readingSensors();
        delay(50);
      }
      break;

    case 'x' :
      readingSensors();
      while ((leftSensorValue != 11) && (rightSensorValue != 11))
      {
        autodistance(leftSensorValue, rightSensorValue);
        readingSensors();
        delay(50);
      }
      break;

  }
}
/****************************************************************************************/



/***************************** Calling PID *************************************************/
void gostraight()
{
  pidcalculator();
}
/******************************************************************************************/



/***************************** Rotate Right 90 *************************************************/
void turnright()
{
  count_m1 = 0;
  count_m2 = 0;
  do
  {
    md.setM1Speed(212);
    md.setM2Speed(-220);

  } while (count_m1 <= 387 || count_m2 <= 387);
  md.setBrakes(400, 400);
}
/**********************************************************************************************/



/***************************** Rotate Left 90 *************************************************/
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
/*********************************************************************************************/



/***************************** Go Straight Block by Block *************************************/
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
/*********************************************************************************************/



/***************************** Get M1 interrupt Count  ****************************************/
/***************************** & Calculate RPM  ************************************************/
void getM1Pulse()
{
  if ( digitalRead(pinM1) == HIGH)
  {
    pulse_prev_m1 = micros();
    count_m1++;
  }
  else
  {
    pulse_now_m1 = micros();
    pulse_prev_m1 = pulse_now_m1 - pulse_prev_m1;
    if ( m1currentrpm != 0 )
    {
      m1currentrpm = (m1currentrpm + (period_to_rpm(pulse_prev_m1))) / 2;
    }
    else
    {
      m1currentrpm = period_to_rpm(pulse_prev_m1);
    }
  }
}
/*********************************************************************************************/



/***************************** Get M2 interrupt Count  ****************************************/
/***************************** & Calculate RPM  ************************************************/
void getM2Pulse()
{
  if ( digitalRead(pinM2) == HIGH)
  {
    pulse_prev_m2 = micros();
    count_m2++;
  }
  else
  {
    pulse_now_m2 = micros();
    pulse_prev_m2 = pulse_now_m2 - pulse_prev_m2;
    if ( m2currentrpm != 0 )
    {
      m2currentrpm = (m2currentrpm + (period_to_rpm(pulse_prev_m2))) / 2;
    }
    else
    {
      m2currentrpm = period_to_rpm(pulse_prev_m2);
    }
  }
}
/*********************************************************************************************/



/********************** Formula to convert Period -> RPM *************************************/
double period_to_rpm(int time)
{
  double n_rpm = 53357.048 / time;
  return n_rpm;
}
/*********************************************************************************************/



/************************** Initial Motors Speed ~ RAM UP***************************************/
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

  md.setM1Speed(131);
  md.setM2Speed(134);
  delay(200);

  md.setM1Speed(181);
  md.setM2Speed(183);
  delay(200);

  md.setM1Speed(216);
  md.setM2Speed(213);
  delay(200);

  md.setM1Speed(m1currentspeed);
  md.setM2Speed(m2currentspeed);
}
/*********************************************************************************************/



/********************************* Moving 1 Block (Hard Code) *******************************/
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
/*********************************************************************************************/



/******************************** Reading All Sensors ***************************************/
void readingSensors()
{
  for (int i = 10; i > 0; i--)
  {
    sensorSR[10 - i] = sR.distance();
    sensorSL[10 - i] = sL.distance();
    sensorSB[10 - i] = sB.distance();
    sensorSM[10 - i] = sM.distance();
  }

  insertionSort(sensorSR);
  insertionSort(sensorSL);
  insertionSort(sensorSB);
  insertionSort(sensorSM);

  //  Serial.print("Left A2 ");
  //  Serial.print(sensorSL[5]);
  leftSensorValue = sensorSL[5];
  //  Serial.print("   ");
  //  Serial.print("Middle A4 ");
  //  Serial.print(sensorSM[5]);
  //  Serial.print("   ");
  //  Serial.print("Right A1 ");
  //  Serial.print(sensorSR[5]);
  rightSensorValue = sensorSR[5];
  //  Serial.println("   ");
  //  Serial.print("Bottom A3 ");
  //  Serial.println(sensorSB[5]);
}
/*********************************************************************************************/



/********************** Insertion Sort <Get Median Value> *********************************/
void insertionSort(double readings[])
{
  double temp;

  int d;
  for (int i = 1 ; i <= 9; i++)
  {
    d = i;
    while (d > 0 && readings[d] < readings[d - 1])
    {
      temp = readings[d];
      readings[d] = readings [d - 1];
      readings[d - 1] = temp;
      d--;
    }
  }
}
/*********************************************************************************************/



/***************************** Auto Alignment <make robot straight> ****************************/
void autoalign(int left, int right)
{
  if ((left - right) < 0)
  {
    md.setSpeeds (-70 , 70);
    delay (100);
    md.setBrakes(400, 400);
  }
  else if ((left - right) > 0)
  {
    md.setSpeeds (70 , -70);
    delay (100);
    md.setBrakes(400, 400);
  }
}
/*******************************************************************************************/



/********** Auto Front <make robot move front or back a bit> *****************************/
void autodistance(int left, int right)
{
  if ((left > 11) && (right > 11))
  {
    md.setSpeeds (70 , 78);
    delay (100);
    md.setBrakes(400, 400);
  }
  else if ((left < 11) && (right < 11))
  {
    md.setSpeeds (-70 , -78);
    delay (100);
    md.setBrakes(400, 400);
  }
}
/*************************************************************************************/
