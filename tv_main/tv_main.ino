#include <tv_defines.h>
#include <TimerOne.h>
#include <TextFinder.h>
#include <Servo.h>
#include <PID_v1.h>
long lastLoop = 0;

// //for PID
// #define loops 1
// #define avg_const 6
// #define response_delay 50
// //torq related
// boolean enable = false;
// int count = 0;
// double total = 0;
// unsigned long last = 0;

// double buffer[loops][avg_const] = {0};
// double torq_readings[loops] = {0};
// double torq[loops] = {0};

// double torq_trigger_low[loops] = {0};
// double torq_trigger_high[loops] = {0};

// double diff_torq_speed[loops] = {0};
// boolean diff_torq_enable[loops] = {false};
// double gap = 0;

// //PID LOOP RELATED(ADAPTIVE LOOP)
// //Define Variables we'll be connecting to
// double Setpoint, Input, Output;

// //Define the aggressive and conservative Tuning Parameters
// double aggKp=4, aggKi=0.2, aggKd=1;
// double consKp=1, consKi=0.05, consKd=0.25;
// //Specify the links and initial tuning parameters for each of the loops
// PID myPID0(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);


int steeringAngle = STEER_STRAIGHT_ANGLE;
Servo steering;

int leftWheelPWM = 0;
int rightWheelPWM = 0;

// for wheel speed
unsigned int leftSpeedCounter = 0;
uint16_t leftWheelRPM = 0;
float leftWheelSpeed = 0;
float leftWheelPWMvalue = 0;

unsigned int rightSpeedCounter = 0;
uint16_t rightWheelRPM = 0;
float rightWheelSpeed = 0;
float rightWheelPWMvalue = 0;

//for yaw
float yaw_ref = 0;
float yaw_err = 0;
int yaw_bias = 0;

void countLeft(){  // counts from the speed sensor
  leftSpeedCounter++;  // increase +1 the counter value
}
void countRight(){  // counts from the speed sensor
  rightSpeedCounter++;  // increase +1 the counter value
}
 
void timerIsr(){
  Timer1.detachInterrupt();  //stop the timer
  // Serial.print("Motor Speed: "); 
  leftWheelRPM = (speedCounter);  // divide by number of holes in disk if any
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
  pinMode(digitalPinToInterrupt(WHEEL_SPEED_LEFT_PIN), RISING);
  Timer1.initialize(LOOP_PERIOD * 1000); // set timer for LOOP_PERIOD
  attachInterrupt(digitalPinToInterrupt(WHEEL_SPEED_LEFT_PIN), count, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(WHEEL_SPEED_RIGHT_PIN), count, RISING);  // increase counter when speed sensor pin goes High

  Timer1.attachInterrupt( timerIsr ); // enable the timer

  //for steering servo
  steering.attach(SERVO_STEER_PIN);

  //for rear wheel motors
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);   
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);

    //to set up individual speed tagets
  torq_trigger_low[0] = 10;
  torq_trigger_high[0] = 50;
  //initialise PID loops  
  for(int b=0; b<loops; b++){
    //PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction) can make an array for the constans later if need different constants
    PIDarray[b] = PID(&temp[b], &fan_speed[b], &temp_trigger_low[b], consKp, consKi, consKd, REVERSE);
    PIDarray[b].SetMode(AUTOMATIC);
  } 

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
    for(int a = 0; a<loops; a++){
      buffer[a][count] = yaw_err;
    }

    //method 2: look up table + PID
    //use a look up table to get the diff torque
    //apply and then switch to PID
    //doesnt work :(

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
  yaw_ref = wheelSpeed * steeringAngle/(WHEELBASE + K_VALUE * pow(wheelSpeed, 2);
}

void calculateYawError(){
  //get avg of IMU
  yaw_err = yaw_ref - (rpy2[1] + rpy1[1])/2;
  if(yaw_err < YAW_ERROR_LIMIT){
      yaw_bias = 0;
      yaw_err = 0;
  }else{
    if(yaw_err > 0){
      yaw_bias = 1;
    }else{
      yaw_bias = -1;
    }
  }
}

void torqueVector(){

  for(int i = 0; i<loops; i++){
        total = 0;
        //get the avg
        //determine if correction is needed
        if(yaw_bias==0){
          return
        }
        
        //distance away from setpoint
        gap = abs(yaw_err);

        if(gap<5){
          //we're close to setpoint, use conservative tuning parameters
          PIDarray[i].SetTunings(consKp, consKi, consKd);
        }
        else{
          //we're far from setpoint, use aggressive tuning parameters
          PIDarray[i].SetTunings(aggKp, aggKi, aggKd);
        }
        PIDarray[i].Compute();
      }
  leftWheelPWMvalue = yaw_bias * diff_torq_speed / MAX_TORQ_DIFF * leftWheelPWMvalue / MAX_PWM; 
  rightWheelPWMvalue = -1 * yaw_bias * diff_torq_speed / MAX_TORQ_DIFF * rightWheelPWMvalue / MAX_PWM;
  analogWrite(leftWheelPWM, leftWheelPWMvalue);   //PWM Speed Control
  analogWrite(rightWheelPWM, rightWheelPWMvalue);   //PWM Speed Control
}