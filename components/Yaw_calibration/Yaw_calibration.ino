#include <TextFinder.h>
#include <Servo.h>

#define PERIOD      50 // [ms]

unsigned long timer = 0;
unsigned long currentTime = 0;

/*** Vars for IMU ***/
const int NUMBER_OF_FIELDS = 3; // how many comma seperated fields we expect  
TextFinder  finder1(Serial1);                                           
float rpy1[NUMBER_OF_FIELDS];    // array holding values for all the fields

TextFinder  finder2(Serial2);                                           
float rpy2[NUMBER_OF_FIELDS];    // array holding values for all the fields

float curr[2] = {0};
float prev[2] = {0};

float yaw=0;
float steer_yaw=0;
// for wheel speed
unsigned long lastDet = 0;
unsigned long nowDet = 0;

float leftWheelRPS = 0;
float leftWheelSpeed = 0;

 void magnet_detect(){
   nowDet = millis();
   leftWheelRPS = 1000.0/((float)(nowDet-lastDet));
   lastDet = nowDet;
 }
 
 //steering
Servo myservo;
int pos = 70;

/************************************************************/
/*** Setup
/************************************************************/
void setup(){
  Serial.begin(57600);  // init the Serial port to print the data to PC
  Serial1.begin(57600); // init the Serial1 port to get data from the IMU
  Serial2.begin(57600); // init the Serial1 port to get data from the IMU
  delay(500);
  initIMUs();

  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)

  myservo.attach(8);
  myservo.write(pos);
}

/************************************************************/
/*** Loop
/************************************************************/
void loop(){
  /************************************************************/
  /*** Request after a specific period for the data
  /************************************************************/
  currentTime = millis();
  if(currentTime - timer >= PERIOD){
    timer = currentTime;
    Serial1.write("#f");
    Serial2.write("#f");
    /************************************************************/
    /*** Get the IMU values
    /************************************************************/

    // the current field being received
    int fieldIndex = 0;
    // search the Serial Buffer as long as the header character is found
    boolean found_HeaderChar = finder1.find("H");

    // for IMU1
    if (found_HeaderChar){
      // Get all 3 values (yaw, pitch, roll) from the Serial Buffer
      while(fieldIndex < NUMBER_OF_FIELDS){
        rpy1[fieldIndex++] = finder1.getFloat();
      }      
      prev[0] = curr[0];
      curr[0] = rpy1[0];
    }

    // for IMU2
    fieldIndex = 0;
    found_HeaderChar = finder2.find("H");
    if (found_HeaderChar){
      // Get all 3 values (yaw, pitch, roll) from the Serial Buffer
      while(fieldIndex < NUMBER_OF_FIELDS){
        rpy2[fieldIndex++] = finder2.getFloat();
      }      
      prev[1] = curr[1];
      curr[1] = rpy2[0];
    }
 	
	yaw = (prev[0]+prev[1]-curr[0]-curr[1])/((float)PERIOD/500.0);
	leftWheelSpeed = 17.55 * leftWheelRPS;
	Serial.print(yaw);
	Serial.print(",");
	Serial.print(leftWheelSpeed);
	Serial.print(",");      
	Serial.println(pos-85);
 	 
  }
}

void initIMUs(){
  // Output angles in TEXT format & Turn off continuous streaming output & Disable error message output
  Serial1.write("#ot#o0#oe0");
  Serial1.flush();
  Serial2.write("#ot#o0#oe0");
  Serial2.flush();
}
