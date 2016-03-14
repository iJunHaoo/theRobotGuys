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

int countList[17] = {280, 592, 879, 1160, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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



/***************************** Get M1 interrupt Count  ****************************************/
void getM1Pulse()
{
  M1tick++;
}
/*********************************************************************************************/


/***************************** Get M2 interrupt Count  ****************************************/
void getM2Pulse()
{
  M2tick++;
}
/*********************************************************************************************/



/***************************** PID Calculator ***********************************************/
double pidcalculator()
{
  double result;
  double m1kp, m1ki, m1kd;
  double m1p, m1i, m1d;

  m1kp = 1.0;
  m1ki = 0.0;
  m1kd = 0.00;

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
    md.setSpeeds(100 + output, 103 - output);
  }

  while ( M2tick < 180 )
  {
    output = pidcalculator();
    md.setSpeeds(200 + output, 204 - output);
  }

  while ( M2tick < 260 )
  {
    output = pidcalculator();
    md.setSpeeds(250 + output, 256 - output);
  }

  while ( M2tick < tickNeeded - 200)
  {
    output = pidcalculator();
    md.setSpeeds(M1Speed + output, M2Speed - output);
  }

  while ( M2tick < tickNeeded)
  {
    output = pidcalculator();
    md.setSpeeds(150 + output, 153 - output);
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

#define rightOFFSET 0


/******************************** Reading All Sensors ***************************************/
void readingSensors()
{
  for (int i = 3; i > 0; i--)
  {
    sensorSR[3 - i] = sR.distance() + rightOFFSET;
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
    sensorSR[3 - i] = sR.distance() + rightOFFSET;
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
    sensorSR[3 - i] = sR.distance() + rightOFFSET;
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

#define alignSpeed 70
#define NEGalignSpeed -70

/***************************** Auto Alignment <make robot straight> ****************************/
void autoalign()
{
  readingAlignSensors();

  for (int i = 0; i < 5; i++)
  {
    if ((leftSensorValue < 15) && (rightSensorValue < 15))
    {
      while (leftSensorValue != rightSensorValue)
      {
        if ((leftSensorValue - rightSensorValue) < 0)
        {
          md.setSpeeds (NEGalignSpeed , alignSpeed);
        }

        else if ((leftSensorValue - rightSensorValue) > 0)
        {
          md.setSpeeds (alignSpeed , NEGalignSpeed);
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

#define distanceValue 10

/********** Auto Front <make robot move front or back a bit> *****************************/
void autodistance()
{
  readingAlignSensors();
  while ((leftSensorValue != distanceValue) && (rightSensorValue != distanceValue))
  {
    if ((leftSensorValue > distanceValue) && (rightSensorValue > distanceValue))
    {
      md.setSpeeds (alignSpeed , alignSpeed);
    }

    else if ((leftSensorValue < distanceValue) && (rightSensorValue < distanceValue))
    {
      md.setSpeeds (NEGalignSpeed , NEGalignSpeed);
    }
    readingAlignSensors();
    delay(50);
  }
  md.setBrakes(400, 400);

}
/*************************************************************************************/
