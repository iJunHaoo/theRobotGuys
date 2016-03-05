int a;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  //md.init();
}

void loop() 
{
 a = Serial.read();
 if(a == 'A')
 {
    Serial.println("B");
 }

}
