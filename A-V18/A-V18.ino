#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <SharpIR2.h>

DualVNH5019MotorShield md;

/***************************** Sensors Variables ***********************************************/
#define sRight A1
#define sLeft A2
#define sBottom A3
#define sMiddle A4
#define sTop A0

int countList[17] = {302, 592, 879, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int sensorSR [10];
int sensorSL [10];
int sensorSB [10];
int sensorSM [10];
int sensorST [10];

SharpIR2 sR (sRight, 10, 99, 1080);
SharpIR2 sL (sLeft, 10, 99, 1080);
SharpIR2 sM (sMiddle, 10, 99, 1080);
SharpIR2 sB (sBottom, 10, 99, 1080);
SharpIR2 sT (sTop, 10, 99, 1080);

int leftSensorValue, rightSensorValue, middleSensorValue, sideFSensor, sideBSensor;
int leftSensorValueInt, rightSensorValueInt, middleSensorValueInt, sideFSensorInt, sideBSensorInt;
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


/************************************* Main Setup **********************************************/
void setup()
{
  Serial.begin(115200);
  //Serial.println("Dual VNH5019 Motor Shield");
  //Serial.println("Hello");
  md.init();

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


/***************************** Get M1 interrupt Count  ****************************************/
/***************************** & Calculate RPM  ***********************************************/
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
/***************************** & Calculate RPM  ***********************************************/
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


/***************************** Motor Straight Setup *********************************************/
void straightsetup()
{
  initial();
  m1rpm2speed();
  m2rpm2speed();
  computeks();
}
/***********************************************************************************************/


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


/***************************** Main Loop *************************************************/
#define amountX 750

void loop()
{
  userInput = Serial.read();

  switch (userInput)
  {
    case 'A' :
      Serial.println("B");
      break;

    case 'B' :
      readingSensors(); //reading all sensors (returning block distance)
      Serial.print(leftSensorValueInt);
      Serial.print(middleSensorValueInt);
      Serial.print(rightSensorValueInt);
      Serial.print(sideFSensorInt);
      Serial.print(sideBSensorInt);
      Serial.println("");
      break;

    case 'C' :
      for (int i = 0; i < 3; i++)
      {
        readingSensors(); //reading all sensors (returning block distance)
        Serial.print("Left: ");
        Serial.print(leftSensorValue);
        Serial.print(" | Middle: ");
        Serial.print(middleSensorValue);
        Serial.print(" | Right: ");
        Serial.print(rightSensorValue);
        Serial.print(" | sideF: ");
        Serial.print(sideFSensor);
        Serial.print(" | sideB: ");
        Serial.print(sideBSensor);
        Serial.println("");
      }
      break;

    case 'q' :
      gostraightblock(1);
      delay(amountX);
      break;

    case 'w' :
      gostraightblock(2);
      delay(amountX);
      break;

    case 'e' :
      gostraightblock(3);
      delay(amountX);
      break;

    case 'r' :
      gostraightblock(4);
      delay(amountX);
      break;

    case 't' :
      gostraightblock(5);
      delay(amountX);
      break;

    case 'y' :
      gostraightblock(6);
      delay(amountX);
      break;

    case 'u' :
      gostraightblock(7);
      delay(amountX);
      break;

    case 'i' :
      gostraightblock(8);
      delay(amountX);
      break;

    case 'o' :
      gostraightblock(9);
      delay(amountX);
      break;

    case 'p' :
      gostraightblock(10);
      delay(amountX);
      break;

    case '[' :
      gostraightblock(11);
      delay(amountX);
      break;

    case ']' :
      gostraightblock(12);
      delay(amountX);
      break;

    case 'l' :
      gostraightblock(13);
      delay(amountX);
      break;

    case 'k' :
      gostraightblock(14);
      delay(amountX);
      break;

    case 'j' :
      gostraightblock(15);
      delay(amountX);
      break;

    case 'h' :
      gostraightblock(16);
      delay(amountX);
      break;

    case 'g' :
      gostraightblock(17);
      delay(amountX);
      break;

    case 'a' :
      turnleft();
      delay(amountX);
      break;

    case 's' :
      turnleft();
      delay(amountX);
      turnleft();
      delay(amountX);
      break;

    case 'd' :
      turnright();
      delay(amountX);
      break;

    case 'z' :
      autoalign();
      delay(amountX);
      break;
  }
}
/****************************************************************************************/


/***************************** Rotate RIGHT 90 *************************************************/
void turnright()
{
  count_m1 = 0;
  count_m2 = 0;
  do
  {
    md.setM1Speed(212);
    md.setM2Speed(-220);
  } while (count_m1 <= 375 || count_m2 <= 375);
  md.setBrakes(400, 400);
}
/**********************************************************************************************/


/***************************** Rotate LEFT 90 *************************************************/
void turnleft()
{
  count_m1 = 0;
  count_m2 = 0;
  do
  {
    md.setM1Speed(-212);
    md.setM2Speed(220);
  } while (count_m1 <= 390 || count_m2 <= 390);
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
  } while (count_m1 <= countList[x - 1] && count_m2 <= countList[x - 1]);
  md.setBrakes(385, 400);
}
/*********************************************************************************************/


/***************************** Calling PID *************************************************/
void gostraight()
{
  pidcalculator();
}
/******************************************************************************************/


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
  md.setM1Speed(38);
  md.setM2Speed(28);
  delay(50);

  md.setM1Speed(38);
  md.setM2Speed(31);
  delay(150);

  md.setM1Speed(43);
  md.setM2Speed(39);
  delay(200);

  md.setM1Speed(132);
  md.setM2Speed(130);
  delay(200);

  md.setM1Speed(187);
  md.setM2Speed(182);
  delay(200);

  md.setM1Speed(214);
  md.setM2Speed(209);
  delay(200);

  do
  {
    md.setM1Speed(252);
    md.setM2Speed(249);
  } while (count_m1 <= (301) && count_m2 <= (301));

  md.setBrakes(400, 395);
  count_m1 = 0;
  count_m2 = 0;
}
/********************************************************************************************/


/******************************** Reading All Sensors ***************************************/
void readingSensors()
{
  for (int i = 3; i > 0; i--)
  {
    sensorSR[3 - i] = sR.distance();
    sensorSL[3 - i] = sL.distance();
    sensorSB[3 - i] = sB.distance();
    sensorSM[3 - i] = sM.distance();
    sensorST[3 - i] = sT.distance();
  }

  insertionSort(sensorSR);
  insertionSort(sensorSL);
  insertionSort(sensorSM);
  insertionSort(sensorSB);
  insertionSort(sensorST);

  leftSensorValue = sensorSL[1];
  rightSensorValue = sensorSR[1];
  middleSensorValue = sensorSM[1];
  sideFSensor = sensorSB[1];
  sideBSensor = sensorST[1];

  leftSensorValueInt = leftBlockFunction(leftSensorValue);
  rightSensorValueInt = rightBlockFunction(rightSensorValue);
  middleSensorValueInt = middleBlockFunction(middleSensorValue);
  sideFSensorInt = sideFBlockFunction(sideFSensor);
  sideBSensorInt = sideBBlockFunction(sideBSensor);
}
/*********************************************************************************************/


/******************************** Reading FRONT Sensors ***************************************/
void readingFrontSensors()
{
  for (int i = 3; i > 0; i--)
  {
    sensorSR[3 - i] = sR.distance();
    sensorSL[3 - i] = sL.distance();
    sensorSM[3 - i] = sM.distance();
  }

  insertionSort(sensorSR);
  insertionSort(sensorSL);
  insertionSort(sensorSM);

  leftSensorValue = sensorSL[1];
  rightSensorValue = sensorSR[1];
  middleSensorValue = sensorSM[1];
}
/*********************************************************************************************/


/******************************** Reading Alignment Sensors ***************************************/
void readingAlignSensors()
{
  for (int i = 3; i > 0; i--)
  {
    sensorSR[3 - i] = sR.distance();
    sensorSL[3 - i] = sL.distance();
  }

  insertionSort(sensorSR);
  insertionSort(sensorSL);

  leftSensorValue = sensorSL[1];
  rightSensorValue = sensorSR[1];
}
/*********************************************************************************************/


/********************** Insertion Sort <Get Median Value> *********************************/
void insertionSort(int readings[])
{
  int temp;

  int d;
  for (int i = 1 ; i <= 3; i++)
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


/***************************** Block Function (LEFT SENSOR)****************************/
int leftBlockFunction(int original) // detect things 3 blocks or less
{
  int blocks;

  if (original < 10)
    return 0; // 8 - 9
  else if (original < 15)
    return 1; // xx - xx MAX XX MIN XX
  else if (original < 27)
    return 2; // 20 - 22 (24) MAX 26 MIN 17
  else if (original < 42)
    return 3; // 34 - 42 MAX 50 MIN 33
  else
    return 9; // more than 40 (SPIKE 37)
}
/*******************************************************************************************/


/***************************** Block Function (RIGHT SENSOR)****************************/
int rightBlockFunction(int original) // detect things 3 blocks or less
{
  int blocks;

  if (original < 9)
    return 0; // xx - xx MAX XX MIN XX
  else if (original < 15)
    return 1; // 10 - 10 MAX 10 MIN 10
  else if (original < 26)
    return 2; // 19 - 20 MAX 20 MIN 19
  else if (original < 36)
    return 3;  // 29 - 34 MAX 29 MIN 34
  else
    return 9; // more than 34 - 37 ( SPIKE MIN 31)
}
/*******************************************************************************************/

/***************************** Block Function (MIDDLE SENSOR)****************************/
int middleBlockFunction(int original) // detect things 3 blocks or less
{
  int blocks;

  if (original < 10)
    return 0; // xx - xx MAX XX MIN XX
  else if (original < 16)
    return 1; // 11 - 11 MAX 11 MIN 11
  else if (original < 29)
    return 2; // 20 - 22 MAX XX MIN XX
  else if (original < 39)
    return 3; // 34 - 36 MAX XX MIN 28
  else
    return 9;  // more than 41
}
/*******************************************************************************************/


/***************************** Block Function (SIDE BACK SENSOR)****************************/
int sideFBlockFunction(int original) // detect things 3 blocks or less
{
  int blocks;

  if (original < 12)
    return 0;
  else if (original < 17)
    return 1; // 12 - 13
  else if (original < 29)
    return 2; // 23 - 25
  else if (original < 34)
    return 3; // 32 - 36 (SPIKE 39)
  else
    return 9; // more than 33
}
/*******************************************************************************************/


/***************************** Block Function (SIDE FRONT SENSOR)****************************/
int sideBBlockFunction(int original) // detect things 3 blocks or less
{
  int blocks;

  if (original < 11)
    return 0;
  else if (original < 15)
    return 1; // 11
  else if (original < 27)
    return 2; // 20 - 22
  else if (original < 37)
    return 3; // 34 - 37
  else
    return 9; // more than 37
}
/*******************************************************************************************/


/***************************** Auto Alignment <make robot straight> ****************************/
void autoalign()
{
  readingAlignSensors();

  for (int i = 0; i < 3; i++)
  {
    if ((leftSensorValue < 15) && (rightSensorValue < 15))
    {
      while (leftSensorValue != rightSensorValue)
      {
        if ((leftSensorValue - rightSensorValue) < 0)
        {
          md.setSpeeds (-60 , 60);
        }

        else if ((leftSensorValue - rightSensorValue) > 0)
        {
          md.setSpeeds (60 , -60);
        }
        readingAlignSensors();
        delay(50);
      }
      md.setBrakes(400, 400);
      autodistance();
    }
  }
}
/*******************************************************************************************/


/********** Auto Front <make robot move front or back a bit> *****************************/
void autodistance()
{
  readingAlignSensors();
  while ((leftSensorValue != 10) && (rightSensorValue != 10))
  {
    if ((leftSensorValue > 9) && (rightSensorValue > 9))
    {
      md.setSpeeds (70 , 70);
    }

    else if ((leftSensorValue < 9) && (rightSensorValue < 9))
    {
      md.setSpeeds (-70 , -70);
    }
    readingAlignSensors();
    delay(50);
  }
  md.setBrakes(400, 400);

}
/*************************************************************************************/
