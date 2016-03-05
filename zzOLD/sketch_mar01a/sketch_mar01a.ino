#include <SharpIR.h>

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

double readingsPin [10];

double readingPin;

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  pinMode(0, OUTPUT);
  analogWrite(0, 0);
  pinMode(5, OUTPUT);
  analogWrite(5, 0);
}

void loop()
{
    for(int i = 10; i > 0; i--)
    {
      sensorSR[10-i] = sR.distance();
      sensorSL[10-i] = sL.distance();
      sensorSB[10-i] = sB.distance();
      sensorSM[10-i] = sM.distance();
    }
    
    insertionSort(sensorSR);
    insertionSort(sensorSL);
    insertionSort(sensorSB);
    insertionSort(sensorSM);
    
    Serial.print(sensorSL[5]);
    Serial.print("   ");
    Serial.print(sensorSM[5]);
    Serial.print("   ");
    Serial.print(sensorSR[5]);
    Serial.println("   ");
    Serial.println(sensorSB[5]);

}

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

