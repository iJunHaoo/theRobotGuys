#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <string.h>

DualVNH5019MotorShield md;
//M1 left wheel, M2 right wheel

int pinM1 = 3;
int pinM2 = 5;
volatile int count = 0, d = 0, grid = 1, reverse = 0, enable = 0;
double m1_kp, m1_ki, m1_kd, m2_kp, m2_ki, m2_kd, m1_k1, m1_k2, m1_k3, m2_k1, m2_k2, m2_k3;

volatile double new_rpm_m1 = 0.0, new_rpm_m2 = 0.0,ideal_rpm1 = 0, ideal_rpm2 = 0, m1_e1, m1_e2, m1_e3, m2_e1, m2_e2, m2_e3;
volatile int  m1_tick = 0, tick = 0, max_tick = 0, check_tick = 0, new_m1_spd = 0, new_m2_spd= 0;
volatile double pulse_prev_m1 = 0.0, pulse_now_m1 = 0.0, pulse_prev_m2 = 0.0, pulse_now_m2 = 0.0, pulse_1_m1 = 0.0, pulse_2_m1 = 0.0, pulse_3_m1 = 0.0, pulse_4_m1 = 0.0, pulse_5_m1 = 0.0 ,pulse_1_m2 = 0.0, pulse_2_m2 = 0.0, pulse_3_m2 = 0.0, pulse_4_m2 = 0.0, pulse_5_m2 = 0.0;

#define BAUD_RATE     115200

/*======================================*/
/*                 Set-Up               */
/*======================================*/

/*One-time execution during its lifetime.*/
void setup()
{
  Serial.begin(BAUD_RATE);            //starts the serial monitor                  //ultrasonic setup
  
  md.init();
  pinMode(pinM1, INPUT);
  pinMode(pinM2, INPUT);

  digitalWrite(pinM1, LOW);
  digitalWrite(pinM2, LOW);

  PCintPort::attachInterrupt(pinM1, getM1Pulse, CHANGE);
  PCintPort::attachInterrupt(pinM2, getM2Pulse, CHANGE);
  
  m1_kp = 4.867; // 4.867; 
  m1_ki = 2.0; // 2.21227; 
  m1_kd = 2.676; 
  
  m2_kp = 6.857; 
  m2_ki = 3.4285;
  m2_kd = 3.4285;
  
  m1_e1 = m1_e2 = m1_e3 = m2_e1 = m2_e2 = m2_e3 = 0.0;
  
  m1_k1 = m1_kp + m1_ki + m1_kd;
  m1_k2 = (-m1_kp) - (2 * m1_kd);
  m1_k3 = m1_kd;
  
  m2_k1 = m2_kp + m2_ki + m2_kd;
  m2_k2 = (-m2_kp) - (2 * m2_kd);
  m2_k3 = m2_kd;
  
  m1_e3 = m1_e2 = m1_e1 = m2_e3 = m2_e2 = m2_e1 = NULL;
  
  straight();
//  turn_left(90);
//  turn_right(90);
//  move_2grid()
}

/*======================================*/
/*              Main Program            */
/*======================================*/

/*Keep looping until the end of time.*/
void loop()
{
//  double prev = micros();
//  testSensors();

  if(enable){
    pid();
  }

 

/* ========Print Both Wheels RPM======== */ 
//  int arpm1 = (pulse_5_m1 + pulse_4_m1 + pulse_3_m1 + pulse_2_m1 + pulse_1_m1)/5;
//  Serial.print(53357.048/arpm1);
//  Serial.print(" ");
//  int arpm2 = (pulse_5_m2 + pulse_4_m2 + pulse_3_m2 + pulse_2_m2 + pulse_1_m2)/5;
//  Serial.println(53357.048/arpm2);
  
  delay(3);

/* ==========Print Loop Time======== */  
//  double tnow = micros() - prev;
//  Serial.println(tnow);  
}



/*======================================*/
/*                 Motors               */
/*======================================*/

void pid()
{
   m1_e3 = m1_e2;
   m1_e2 = m1_e1;
   
   m2_e3 = m2_e2;
   m2_e2 = m2_e1;
   
   double avg_m1_pulse = (pulse_5_m1 + pulse_4_m1 + pulse_3_m1 + pulse_2_m1 + pulse_1_m1)/5;
   double avg_m2_pulse = (pulse_5_m2 + pulse_4_m2 + pulse_3_m2 + pulse_2_m2 + pulse_1_m2)/5;
   
   double prev_m1_rpm = (53357.048/avg_m1_pulse);
   double prev_m2_rpm = (53357.048/avg_m2_pulse);
   
//   Serial.println(prev_m1_rpm);
   
   m1_e1 = ideal_rpm1 - prev_m1_rpm;
   m2_e1 = ideal_rpm2 - prev_m2_rpm;
   
   if(m1_e3 != NULL && m2_e3 != NULL){
     new_rpm_m1 = prev_m1_rpm + m1_k1*m1_e1 + m1_k2*m1_e2 + m1_k3*m1_e3;
     new_m1_spd = (int)(((new_rpm_m1 + 9.5714)/0.3785) + 0.5);
     //(int)(((new_rpm_m1 + 9.5714)/0.36) + 0.5)

     new_rpm_m2 = prev_m2_rpm + m2_k1*m2_e1 + m2_k2*m2_e2 + m2_k3*m2_e3;
     new_m2_spd = (int)(((new_rpm_m2 + 6.7143)/0.42) + 0.5);
    

       md.setM1Speed(new_m1_spd);
       md.setM2Speed(new_m2_spd);
    
   } 
}

void getM1Pulse(){
  if( digitalRead(pinM1) == HIGH){
    pulse_prev_m1 = micros();
    if(check_tick && (tick == max_tick)){
       brake();
       check_tick = 0;
    }else{
      ++tick;
    }
  }
  else{
    pulse_now_m1 = micros();
    pulse_5_m1 = pulse_4_m1;
    pulse_4_m1 = pulse_3_m1;
    pulse_3_m1 = pulse_2_m1;
    pulse_2_m1 = pulse_1_m1;
    pulse_1_m1 = pulse_now_m1 - pulse_prev_m1;
  }
}

void getM2Pulse(){
  if( digitalRead(pinM2) == HIGH){
    pulse_prev_m2 = micros(); 
  }
  else{
    pulse_now_m2 = micros();
    pulse_5_m2 = pulse_4_m2;
    pulse_4_m2 = pulse_3_m2;
    pulse_3_m2 = pulse_2_m2;
    pulse_2_m2 = pulse_1_m2;
    pulse_1_m2 = pulse_now_m2 - pulse_prev_m2;
  }
}

void straight()
{
  ideal_rpm1 = ideal_rpm2 = 100.0;
  md.setM1Speed(10);
  md.setM2Speed(10);
  delay(50);
  md.setM1Speed(30);
  md.setM2Speed(30);
  delay(100);
  md.setM1Speed(80);
  md.setM2Speed(80);
  delay(100);  
  md.setM1Speed(120);
  md.setM2Speed(120);
  delay(100); 
  md.setM1Speed(180);
  md.setM2Speed(180);
  delay(100); 
  md.setM1Speed(230);
  md.setM2Speed(230);
  delay(100);
  enable = 1;
}

void brake(){
  enable = 0;
  ideal_rpm1 = ideal_rpm2 = 0.0;
  md.setM1Speed(100);
  md.setM2Speed(100);
  delay(100);  
  md.setM1Speed(80);
  md.setM2Speed(80);
  delay(100);
  md.setM1Speed(50);
  md.setM2Speed(50);
  delay(100);
  md.setM1Speed(20);
  md.setM2Speed(20);
  delay(100);  
  md.setM1Speed(10);
  md.setM2Speed(10);
  delay(100);
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(100);
}

void turn_left(int degree){
  check_tick = 1;
  ideal_rpm1 = 70.0;
  ideal_rpm2 = 70.0;
  if(degree == 90) max_tick = 327; // 90degree : 403 | 180degree : 735 | 360degree : 1530 | 720degree :  3126
  else if(degree == 180) max_tick = 735;
  else if(degree == 360) max_tick = 1530;
  else max_tick = 3126;
  tick = 0;
  md.setM1Speed(10);
  md.setM2Speed(-10);
  delay(100);
  md.setM1Speed(40);
  md.setM2Speed(-40);
  delay(100);
  md.setM1Speed(70);
  md.setM2Speed(-70);
  delay(100);  
  md.setM1Speed(100);
  md.setM2Speed(-100);
  delay(100); 
  md.setM1Speed(150);
  md.setM2Speed(-150);
  delay(100); 
  enable = 1;
}

void turn_right(int degree){
  check_tick = 1;
  ideal_rpm1 = 70.0;
  ideal_rpm2 = 70.0;
  if(degree == 90) max_tick = 327; // 90degree : 403 | 180degree : 735 | 360degree : 1530 | 720degree :  3126
  else if(degree == 180) max_tick = 735;
  else if(degree == 360) max_tick = 1530;
  else max_tick = 3126;
  tick = 0;
  md.setM1Speed(10);
  md.setM2Speed(-10);
  delay(100);
  md.setM1Speed(40);
  md.setM2Speed(-40);
  delay(100);
  md.setM1Speed(70);
  md.setM2Speed(-70);
  delay(100);  
  md.setM1Speed(100);
  md.setM2Speed(-100);
  delay(100); 
  md.setM1Speed(150);
  md.setM2Speed(-150);
  delay(100); 
  enable = 1;
}

void move_2grid(){
  check_tick = 1;
  grid = 0;
  ideal_rpm1 = ideal_rpm2 = 100.0;
  max_tick = 580; // 2grid: 600, 1grid : 300
  tick = 0;
  md.setM1Speed(10);
  md.setM2Speed(10);
  delay(10);
  md.setM1Speed(30);
  md.setM2Speed(30);
  delay(50);
  md.setM1Speed(50);
  md.setM2Speed(50);
  delay(50);    
}

void drift(){
  check_tick = 1;
  ideal_rpm1 = 38.0;
  ideal_rpm2 = 29.0;
  max_tick = 800; // 2grid: 600, 1grid : 300
  tick = 0;
  md.setM1Speed(10);
  md.setM2Speed(10);
  delay(10);
  md.setM1Speed(30);
  md.setM2Speed(30);
  delay(50);
  md.setM1Speed(80);
  md.setM2Speed(80);
  delay(50);  
}








// 1 March
