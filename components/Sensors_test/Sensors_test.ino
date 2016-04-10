
#define LOOP_PERIOD 1000
unsigned long now = 0;
unsigned long lastLoop = 0;

 volatile byte leftSpeedCounter = 0;
 float leftWheelRPM = 0;
float leftWheelSpeed = 0;
 void setup(){
   Serial.begin(115200);
   pinMode(2, INPUT);
   attachInterrupt(digitalPinToInterrupt(2), magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)
 }
 void loop(){
//  Serial.println(digitalRead(2));
  now = millis();
   if (now-lastLoop >= LOOP_PERIOD) { 
    lastLoop = now;
     leftWheelRPM = (float)leftSpeedCounter*60000.0/(float)LOOP_PERIOD;     
//    leftWheelSpeed = 20.0 * leftWheelRPM/60.0;
     leftSpeedCounter = 0; 
     Serial.print("RPM: ");
     Serial.println(leftWheelRPM);
//     Serial.print("Speed: ");   
//     Serial.println(leftWheelSpeed);
   }
 }
 void magnet_detect(){
   leftSpeedCounter++;
   Serial.println("det");
 }
