
#define LOOP_PERIOD 50
unsigned long now = 0;
unsigned long lastLoop = 0;
unsigned long lastDet = 0;
unsigned long nowDet = 0;

volatile byte leftSpeedCounter = 0;
float leftWheelRPS = 0;
float leftWheelSpeed = 0;

 void magnet_detect(){
   nowDet = millis();
   leftWheelRPS = 1000.0/((float)(nowDet-lastDet));
   lastDet = nowDet;
 }

 void setup(){
   Serial.begin(115200);
   pinMode(2, INPUT);
   attachInterrupt(digitalPinToInterrupt(2), magnet_detect, FALLING);//Initialize the intterrupt pin (Arduino digital pin 2)
 }
 void loop(){
  now = millis();
   if (now-lastLoop >= LOOP_PERIOD) { 
    lastLoop = now;   
    leftWheelSpeed = 17.55 * leftWheelRPS;
     leftSpeedCounter = 0; 
     Serial.print("RPS: ");
     Serial.print(leftWheelRPS);
     Serial.print(" , Speed: ");   
     Serial.println(leftWheelSpeed);
   }
 }
