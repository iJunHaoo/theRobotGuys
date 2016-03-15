#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
//#include <SharpIR2.h>

DualVNH5019MotorShield md;

/***************************** Sensors Variables ***********************************************/
#define sRight A1
#define sLeft A2
#define sBottom A3
#define sMiddle A4
#define sTop A0

int countList[17] = {285, 582, 878, 1160, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
  turnright();
  delay(750);
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
  double result, last;
  double m1kp, m1ki, m1kd;
  double m1p, m1i, m1d;

  m1kp = 1.0;
  m1ki = 0.0;
  m1kd = 0.00;

  tickDifference = M2tick - M1tick;
  integral += tickDifference;

  m1p = tickDifference * m1kp;
  m1i = integral * m1ki;
  m1d = ( last - M2tick ) * m1kd;

  result = m1p + m1i + m1d;

  last = M2tick;
  
  return result;
}
/*********************************************************************************************/



/***************************** Go Straight Block by Block *************************************/
void gostraightblock(int x)
{
  double output = 0;
  M1tick = 0;
  M2tick = 0;
  tickDifference = 0;
  integral = 0;

  double M2Ramp2, M2Ramp3, M2DeRamp;
  
  switch (x)
  {
    case 1 : tickNeeded = countList[0];
              M2Ramp2 = 205;
              M2Ramp3 = 255;
              M2DeRamp = 153;
      break;

    case 2 : tickNeeded = countList[1];
              M2Ramp2 = 205;
              M2Ramp3 = 257;
              M2DeRamp = 170;
      break;

    case 3 : tickNeeded = countList[2];
              M2Ramp2 = 203;
              M2Ramp3 = 255;
              M2DeRamp = 165;
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

  //Debug
  // tickNeeded = 880;
  
  while ( M2tick < 100 )
  {
    output = pidcalculator();
    md.setSpeeds(100 + output, 100 - output);
  }

  while ( M2tick < 180 )
  {
    output = pidcalculator();
    md.setSpeeds(200 + output, M2Ramp2 - output);
  }

  while ( M2tick < 260 )
  {
    output = pidcalculator();
    md.setSpeeds(250 + output, M2Ramp3 - output);
  }

  while ( M2tick < tickNeeded - 200)
  {
    output = pidcalculator();
    md.setSpeeds(M1Speed + output, M2Speed - output);
  }

  while ( M2tick < tickNeeded)
  {
    output = pidcalculator();
    md.setSpeeds(150 + output, M2DeRamp - output);
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
  } while (M1tick <= 380 || M2tick <= 380);
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





