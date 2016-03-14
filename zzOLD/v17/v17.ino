#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <SharpIR.h>

DualVNH5019MotorShield md;

/***************************** Sensors Variables ***********************************************/
#define sRight A1
#define sLeft A2
#define sBottom A3
#define sMiddle A4
#define sTop A0

int countList[17] = {302, 592, 879, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double sensorSR [10];
double sensorSL [10];
double sensorSB [10];
double sensorSM [10];
double sensorST [10];

SharpIR sR (sRight , 1080);
SharpIR sL (sLeft , 1080);
SharpIR sB (sBottom , 1080);
SharpIR sM (sMiddle , 1080);
SharpIR sT (sTop , 1080);

double leftSensorValue, rightSensorValue, middleSensorValue, sideFSensor, sideBSensor;
int leftSensorValueInt, rightSensorValueInt, middleSensorValueInt, sideFSensorInt, sideBSensorInt;
/**********************************************************************************************/


/*****************************  Motor Variables  ***********************************************/
int pinM1 = 3;
int pinM2 = 5;
double tickDifference = 0.0;
double integral = 0.0;
double tickNeeded = 0.0;

// ------ M1 ----- //
int M1tick = 0;
double M1Speed;

// ----- M2 ----- //
int M2tick = 0;
double M2Speed;
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

  M1Speed = 250;
  M2Speed = 250;

  digitalWrite(pinM1, LOW);
  digitalWrite(pinM2, LOW);

  PCintPort::attachInterrupt(pinM1, getM1Pulse, RISING);
  PCintPort::attachInterrupt(pinM2, getM2Pulse, RISING);
}
/*********************************************************************************************/


/***************************** Main Loop *************************************************/
#define amountX 750

void loop()
{
//  userInput = Serial.read();
//
//  switch (userInput)
//  {
//    case 'A' :
//      Serial.println("B");
//      break;
//
//    case 'B' :
//      readingSensors(); //reading all sensors (returning block distance)
//      Serial.print(leftSensorValueInt);
//      Serial.print(middleSensorValueInt);
//      Serial.print(rightSensorValueInt);
//      Serial.print(sideFSensorInt);
//      Serial.print(sideBSensorInt);
//      Serial.println("");
//      break;
//
//    case 'C' :
//      for (int i = 0; i < 3; i++)
//      {
//        readingSensors(); //reading all sensors (returning block distance)
//        Serial.print("Left: ");
//        Serial.print(leftSensorValue);
//        Serial.print(" | Middle: ");
//        Serial.print(middleSensorValue);
//        Serial.print(" | Right: ");
//        Serial.print(rightSensorValue);
//        Serial.print(" | sideF: ");
//        Serial.print(sideFSensor);
//        Serial.print(" | sideB: ");
//        Serial.print(sideBSensor);
//        Serial.println("");
//      }
//      break;
//
//    case 'q' :
//      gostraightblock(1);
//      delay(amountX);
//      break;
//
//    case 'z' :
//      autoalign();
//      autoalign();
//      delay(amountX);
//      break;
//  }

    gostraightblock(1);
    delay(amountX);
    
}
/****************************************************************************************/



/***************************** Get M1 interrupt Count  ****************************************/
void getM1Pulse()
{
  M1tick++;
}
/*********************************************************************************************/


/***************************** Get M2 interrupt Count  ****************************************/
void getM2Pulse()
{
  M2tick++
}
/*********************************************************************************************/



/***************************** PID Calculator ***********************************************/
double pidcalculator()
{
  double result;
  double m1kp, m1ki, m1kd;
  double m1p, m1i, m1d;

  m1kp = 50;
  m1ki = 0.1;
  m1kd = 0.01;

  tickDifference = M2tick - M1tick;
  integral += tickDifference;

  m1p = tickDifference * m1kp;
  m1i = integral * m1ki;
  m1d = ( 0 - M2tick ) * m1kd;

  result = m1p + m1i + m1d;

  return result;
}
/*********************************************************************************************/



/***************************** Go Straight Block by Block *************************************/
void gostraightblock(int x)
{
  int output = 0;
  M1tick = 0;
  M2tick = 0;
  tickDifference = 0;
  integral = 0;

  switch (x)
  {
    case 1 : tickNeeded = countList[0];
      break;

    case 2 : tickNeeded = countList[1];
      break;

    case 3 : tickNeeded = countList[2];
      break;

    case 4 : tickNeeded = countList[3];
      break;

    case 5 : tickNeeded = countList[4];
      break;

    case 6 : tickNeeded = countList[5];
      break;

    case 7 : tickNeeded = countList[6];
      break;

    case 8 : tickNeeded = countList[7];
      break;

    case 9 : tickNeeded = countList[8];
      break;

    case 10 : tickNeeded = countList[9];
      break;

    case 11 : tickNeeded = countList[10];
      break;

    case 12 : tickNeeded = countList[11];
      break;

    case 13 : tickNeeded = countList[12];
      break;

    case 14 : tickNeeded = countList[13];
      break;

    case 15 : tickNeeded = countList[14];
      break;

    case 16 : tickNeeded = countList[15];
      break;

    case 17 : tickNeeded = countList[16];
      break;

  }

  while ( M2tick < 100 )
  {
    output = pidcalculator();
    md.setSpeeds(100 + output, 100 - output);
  }

  while ( M2tick < 180 )
  {
    output = pidcalculator();
    md.setSpeeds(200 + output, 200 - output);
  }

  while ( M2tick < 260 )
  {
    output = pidcalculator();
    md.setSpeeds(250 + output, 250 - output);
  }

  while ( M2tick < tickNeeded - 200)
  {
    output = pidcalculator();
    md.setSpeeds(M1Speed + output, M2Speed - output);
  }

  while ( M2tick < tickNeeded)
  {
    output = pidcalculator();
    md.setSpeeds(150 + output, 150 - output);
  }

  md.setBrakes(385, 400);
  delay(80);
  md.setBrakes(0, 0);
}
/*********************************************************************************************/



/***************************** Rotate RIGHT 90 *************************************************/
void turnright()
{
  M1tick = 0;
  M2tick = 0;
  do
  {
    md.setM1Speed(212);
    md.setM2Speed(-220);
  } while (M1tick <= 375 || M2tick <= 375);
  md.setBrakes(400, 400);
}
/**********************************************************************************************/


/***************************** Rotate LEFT 90 *************************************************/
void turnleft()
{
  M1tick = 0;
  M2tick = 0;
  do
  {
    md.setM1Speed(-212);
    md.setM2Speed(220);
  } while (M1tick <= 390 || M2tick <= 390);
  md.setBrakes(400, 400);
}
/*********************************************************************************************/




/******************************** Reading All Sensors ***************************************/
void readingSensors()
{
  for (int i = 5; i > 0; i--)
  {
    sensorSR[5 - i] = sR.distance();
    sensorSL[5 - i] = sL.distance();
    sensorSB[5 - i] = sB.distance();
    sensorSM[5 - i] = sM.distance();
    sensorST[5 - i] = sT.distance();
  }

  insertionSort(sensorSR);
  insertionSort(sensorSL);
  insertionSort(sensorSM);
  insertionSort(sensorSB);
  insertionSort(sensorST);

  leftSensorValue = sensorSL[2];
  rightSensorValue = sensorSR[2];
  middleSensorValue = sensorSM[2];
  sideFSensor = sensorSB[2];
  sideBSensor = sensorST[2];

  leftSensorValueInt = block2Function(leftSensorValue);
  rightSensorValueInt = block2Function(rightSensorValue);
  middleSensorValueInt = block2Function(middleSensorValue);
  sideFSensorInt = block2Function(sideFSensor);
  sideBSensorInt = block2Function(sideBSensor);
}
/*********************************************************************************************/


/******************************** Reading FRONT Sensors ***************************************/
void readingFrontSensors()
{
  for (int i = 5; i > 0; i--)
  {
    sensorSR[5 - i] = sR.distance();
    sensorSL[5 - i] = sL.distance();
    sensorSM[5 - i] = sM.distance();
  }

  insertionSort(sensorSR);
  insertionSort(sensorSL);
  insertionSort(sensorSM);

  leftSensorValue = sensorSL[2];
  rightSensorValue = sensorSR[2];
  middleSensorValue = sensorSM[2];
}
/*********************************************************************************************/


/******************************** Reading Alignment Sensors ***************************************/
void readingAlignSensors()
{
  for (int i = 5; i > 0; i--)
  {
    sensorSR[5 - i] = sR.distance();
    sensorSL[5 - i] = sL.distance();
  }

  insertionSort(sensorSR);
  insertionSort(sensorSL);

  leftSensorValue = sensorSL[2];
  rightSensorValue = sensorSR[2];
}
/*********************************************************************************************/


/********************** Insertion Sort <Get Median Value> *********************************/
void insertionSort(double readings[])
{
  double temp;

  int d;
  for (int i = 1 ; i <= 5; i++)
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


/***************************** Block Function ****************************/
int block3Function(double original) // detect things 3 blocks or less
{
  int blocks;

  blocks = (int)original / 10;

  if (blocks < 4)
  {
    return blocks;
  }

  else
  {
    return 9;
  }
}
/*******************************************************************************************/


/***************************** Block Function (Side Sensor) ****************************/
int block2Function(double original) //detech things 2 blocks or less
{
  int blocks;

  blocks = (int)original / 10;

  if (blocks < 3)
  {
    return blocks;
  }

  else
  {
    return 9;
  }
}
/*******************************************************************************************/


/***************************** Auto Alignment <make robot straight> ****************************/
void autoalign()
{
  readingAlignSensors();

  if ((leftSensorValue > 20) && (rightSensorValue > 20) && (middleSensorValue > 20)) return;

  for (int i = 0; i < 3; i++)
  {
    if ((leftSensorValue < 20) && (rightSensorValue < 20))
    {
      while (leftSensorValue != rightSensorValue)
      {
        //        Serial.println("LEFT RIGHT - ");
        //        Serial.print(leftSensorValue);
        //        Serial.print(" | ");
        //        Serial.print(middleSensorValue);
        //        Serial.print(" | ");
        //        Serial.println(rightSensorValue);
        if ((leftSensorValue - rightSensorValue) < 0)
        {
          md.setSpeeds (-70 , 70);
        }

        else if ((leftSensorValue - rightSensorValue) > 0)
        {
          md.setSpeeds (70 , -70);
        }
        readingAlignSensors();
        delay(50);
      }
      md.setBrakes(400, 400);
    }
    autodistance();
  }
}
/*******************************************************************************************/


/********** Auto Front <make robot move front or back a bit> *****************************/
void autodistance()
{
  readingFrontSensors();
  while ((leftSensorValue != 11.0) && (rightSensorValue != 11.0) && (middleSensorValue != 11.0))
  {
    if ((leftSensorValue > 11.0) && (rightSensorValue > 11.0) && (middleSensorValue > 11.0))
    {
      md.setSpeeds (70 , 70);
    }

    else if ((leftSensorValue < 11.0) && (rightSensorValue < 11.0) && (middleSensorValue < 11.0))
    {
      md.setSpeeds (-70 , -70);
    }
    readingFrontSensors();
    delay(50);
  }
  md.setBrakes(400, 400);

}
/*************************************************************************************/
