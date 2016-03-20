#include <TimerOne.h>

#define LOOP_PERIOD 100
unsigned int speedCounter = 0;
uint16_t wheelRPM = 0;
 
void docount(){  // counts from the speed sensor
  speedCounter++;  // increase +1 the counter value
} 
 
void timerIsr(){
  Timer1.detachInterrupt();  //stop the timer
  // Serial.print("Motor Speed: "); 
  wheelRPM = (speedCounter / 20);  // divide by number of holes in Disc
  // Serial.print(rotation,DEC);  
  // Serial.println(" Rotation per seconds"); 
  speedCounter=0;  //  reset counter to zero
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}
long lastLoop = 0;

void setup(){
  Serial.begin(115200);
  Serial2.begin(115200);

  //for speed measurement
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(0, docount, RISING);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop(){
  long now = millis();
  if(now-lastLoop >= LOOP_PERIOD ){
    lastLoop = now;
  //loop every 100 ms

  //get sensor data
  getYaw();
    //wheel speed

    //steering angle
  getSteeringAngle();
    //yaw
   //write sensor data to SD

    //calculate intended yaw
    calculateYaw();
    //use sensor for actual yaw
    //calculate error
    calculateYawError();

    //method 1: just pid
    //use pid to calculate differential torque,
    // + for more right, - for more left
    //factor that into the wheel speed

    //method 2: look up table + PID
    //use a look up table to get the diff torque
    //apply and then switch to PID

   //write error value to SD
  }
}

void getYaw(){
  
}
void getSteeringAngle(){

}

void calculateYaw(){
  
}

void calculateYawError(){
  
}

