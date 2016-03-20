#include <tv_defines.h>
#include <TimerOne.h>
#include <TextFinder.h>
#include <Servo.h>

long lastLoop = 0;

int steeringAngle = STEER_STRAIGHT_ANGLE;
Servo steering;

int leftWheelPWM = 0;
int rightWheelPWM = 0;

// for wheel speed
unsigned int speedCounter = 0;
uint16_t wheelRPM = 0;
float wheelSpeed = 0;

void count(){  // counts from the speed sensor
  speedCounter++;  // increase +1 the counter value
} 
 
void timerIsr(){
  Timer1.detachInterrupt();  //stop the timer
  // Serial.print("Motor Speed: "); 
  wheelRPM = (speedCounter);  // divide by number of holes in disk if any
  // Serial.print(rotation,DEC);  
  // Serial.println(" Rotation per seconds"); 
  speedCounter=0;  //  reset counter to zero
  Timer1.attachInterrupt(timerIsr);  //enable the timer
}

// for IMU1
TextFinder  finder1(Serial2); 
float rpy1[NUMBER_OF_FIELDS];    // array holding values for all the fields


// for IMU2
TextFinder  finder2(Serial3); 
float rpy2[NUMBER_OF_FIELDS];    // array holding values for all the fields


void setup(){
  //init serial
  Serial.begin(115200); //for serial output
  Serial2.begin(57600); //for IMU1
  Serial3.begin(57600); //for IMU2

  //for speed measurement
  Timer1.initialize(LOOP_PERIOD * 1000); // set timer for LOOP_PERIOD
  attachInterrupt(0, count, RISING);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer

  //for steering servo
  steering.attach(SERVO_STEER_PIN);

  //for rear wheel motors
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);   
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT); 

  delay(500);
  initIMUs();
}

void loop(){
  long now = millis();
  //loop every 100 ms
  if(now-lastLoop >= LOOP_PERIOD ){
    lastLoop = now;

    //get sensor data
    getYaw();

    //wheel speed
    wheelSpeed = WHEEL_CIRCUMFERENCE * wheelRPM * (1000/LOOP_PERIOD)
    //steering angle

    getSteeringAngle(); // for later if wanna input steering angle
    steering.write(steeringAngle);
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

    torqueVector();

   //write error value to SD
  }
}

/********************************/
/*** Initialize IMUs
/********************************/

void initIMUs()
{
  // Output angles in TEXT format & Turn off continuous streaming output & Disable error message output
  Serial2.write("#ot#o0#oe0");
  Serial3.write("#ot#o0#oe0");
  Serial2.flush();
  Serial3.flush();
}


void getYaw(){
    Serial2.write("#f");
    Serial3.write("#f");
    
    /************************************************************/
    /*** Get the IMU values from both IMUs
    /************************************************************/

    // the current field being received
    int fieldIndex = 0;            

    // search the Serial Buffer as long as the header character is found
    boolean found_HeaderChar = finder1.find("H");

    if (found_HeaderChar)
    {
      // Get all 3 values (yaw, pitch, roll) from the Serial Buffer
      while(fieldIndex < IMU_FIELDS)
      {
        rpy1[fieldIndex++] = finder1.getFloat();
      }
    }
    
    // reset the current field being received
    fieldIndex = 0;            

    // search the Serial Buffer as long as the header character is found
    found_HeaderChar = finder2.find("H");

    if (found_HeaderChar)
    {
      // Get all 3 values (yaw, pitch, roll) from the Serial Buffer
      while(fieldIndex < IMU_FIELDS)
      {
        rpy2[fieldIndex++] = finder2.getFloat();
      }
    }  
}

void calculateYaw(){
  //use steeringAngle and wheelSpeed
}

void calculateYawError(){
  
}

void torqueVector(){
  analogWrite(leftWheelPWM, value);   //PWM Speed Control
  analogWrite(rightWheelPWM, value);   //PWM Speed Control
}