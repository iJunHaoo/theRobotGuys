#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>

DualVNH5019MotorShield md;

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

/*********************************** SET UP ***************************************/
void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  pinMode(0, OUTPUT);
  analogWrite(0, 0);
  pinMode(5, OUTPUT);
  analogWrite(5, 0);
}
/**********************************************************************************/


int abc;
/***************************** Loop ***********************************************/
void loop()
{
  readingSensors();
  Serial.print("Left Sensor Value ");
  Serial.print(leftSensorValue);
  Serial.println("");
  Serial.print("Right Sensor Value ");
  Serial.print(rightSensorValue);
  Serial.println("");
  
  abc = Serial.read();
  if (abc == 'A')
  {
    while(leftSensorValue != rightSensorValue)
    {
    readingSensors();
    autoalign(leftSensorValue, rightSensorValue);
    delay(100);
    }
  }
  else if (abc == 'B')
  {
    while((leftSensorValue != 12) && (rightSensorValue != 12))
    {
      readingSensors();
      autofront(leftSensorValue, rightSensorValue);
      delay(50);
    }
  }
  else if (abc == 'C')
  {
    while((leftSensorValue != 12) && (rightSensorValue != 12))
    {
      readingSensors();
      autoback(leftSensorValue, rightSensorValue);
      delay(50);
    }
  }
  
}
/**********************************************************************************/



// Reading All Sensors
/**********************************************************************************/
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

  Serial.print("Left A2 ");
  Serial.print(sensorSL[5]);
  leftSensorValue = sensorSL[5];
  Serial.print("   ");
  Serial.print("Middle A4 ");
  Serial.print(sensorSM[5]);
  Serial.print("   ");
  Serial.print("Right A1 ");
  Serial.print(sensorSR[5]);
  rightSensorValue = sensorSR[5];
  Serial.println("   ");
  Serial.print("Bottom A3 ");
  Serial.println(sensorSB[5]);
}
/**********************************************************************************/



// Insertion Sort <Get Median Value>
/**********************************************************************************/
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
/**********************************************************************************/



// Auto Alignment <make robot straight>
/**********************************************************************************/
void autoalign(int left, int right)
{
  if ((left - right) < 0)
  {
    md.setSpeeds (-70 , 70);
    delay (100);
    md.setBrakes(400,400);
  }
  else if ((left - right) > 0)
  {
    md.setSpeeds (70 , -70);
    delay (100);
    md.setBrakes(400,400);
  }

}
/**********************************************************************************/



// Auto Front <make robot move front a bit>
/**********************************************************************************/
void autofront(int left, int right)
{
  if ((left > 12) && (right > 12))
  {
    md.setSpeeds (70 , 70);
    delay (100);
    md.setBrakes(400,400);
  }
}
/**********************************************************************************/




// Auto Back <make robot move back a bit>
/**********************************************************************************/
void autoback(int left, int right)
{
  if ((left < 12) && (right < 12))
  {
    md.setSpeeds (-70 , -70);
    delay (100);
    md.setBrakes(400,400);
  }
}
/**********************************************************************************/

