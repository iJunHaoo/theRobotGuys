#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  
  //Input of Encoder for M1
  pinMode(3, INPUT);
  //Input of Encoder for M2
  pinMode(5, INPUT);
  
  md.init();
}

void loop()
{
  long startTime, elapsedTime;
  int i;
  double pulse, rps, rpm;

  //Set both motor speed to 300
  //Followed by 3 secs delay
  md.setM1Speed(300);
  md.setM2Speed(-300);
  delay(3000);

  //Calculate RPM and plotting at against time
  startTime = millis();
  Serial.println("Speed 300");
  for(i = 0; i < 50; i++)
  {
    pulse = pulseIn(5,HIGH);
    rps = (pulse * 2 * 562.25) / 1000000;
    rpm = 60 / rps;
    Serial.println(rpm);
    elapsedTime = millis() - startTime;
    Serial.println(elapsedTime);
    delay(20);  
  }
  
  //Set both motor speed to 250
  //Without delay
  md.setM1Speed(250);
  md.setM2Speed(-250);

  //Calculate RPM and plotting at against time
  Serial.println("Speed 250");
  for(i = 0; i < 50; i++)
  {
    pulse = pulseIn(5,HIGH);
    rps = (pulse * 2 * 562.25) / 1000000;
    rpm = 60 / rps;
    Serial.println(rpm);
    elapsedTime = millis() - startTime;
    Serial.println(elapsedTime);
    delay(20);  
  }
}
  
