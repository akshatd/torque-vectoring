/**************************** INFO ********************************/

// This code expects a message in the format: H 12.00,-345.00,678.00

/******************************************************************/

#include <TextFinder.h>

/*** Defines the frequency at which the data is requested ***/
/*** frequency f=1/T, T=period; ie. 100ms --> f=10Hz, 200ms --> f=5Hz ***/
#define PERIOD      20 // [ms]

/*** Vars for IMU ***/
TextFinder  finder(Serial3);  
const int NUMBER_OF_FIELDS = 3; // how many comma seperated fields we expect                                           
float rpy[NUMBER_OF_FIELDS];    // array holding values for all the fields

/************************************************************/
/*** Setup
/************************************************************/
void setup()
{
  Serial.begin(57600);  // init the Serial port to print the data to PC
  Serial3.begin(57600); // init the Serial3 port to get data from the IMU

  delay(500);

  initIMU();
}

/************************************************************/
/*** Loop
/************************************************************/
void loop()
{
  // print manager timer
  static unsigned long timer = 0;
  static unsigned long currentTime = 0;

  /************************************************************/
  /*** Request after a specific period for the data
  /************************************************************/
  currentTime = millis();
  if(currentTime - timer >= PERIOD)
  {
    // Request one output frame from the IMU
    // #f only requests one reply, replies are still bound to the internal 20ms (50Hz) time raster.
    // So worst case delay that #f can add is 19.99ms.
    Serial3.write("#f");
    
    /************************************************************/
    /*** Get the IMU values
    /************************************************************/

    // the current field being received
    int fieldIndex = 0;            

    // search the Serial Buffer as long as the header character is found
    boolean found_HeaderChar = finder.find("H");

    if (found_HeaderChar)
    {
      // Get all 3 values (yaw, pitch, roll) from the Serial Buffer
      while(fieldIndex < NUMBER_OF_FIELDS)
      {
        rpy[fieldIndex++] = finder.getFloat();
      }
    }
    
    /************************************************************/
    /*** Print out the values
    /*** Format: yaw, pitch, roll, left_Encoder, right_Encoder
    /************************************************************/
    if (found_HeaderChar)
    {
      // print Interval
      Serial.print(currentTime - timer);
      Serial.print(",");

      // print IMU values
      for(fieldIndex=0; fieldIndex < NUMBER_OF_FIELDS; fieldIndex++)
      {
        Serial.print(rpy[fieldIndex]);
        Serial.print(",");
      }
      Serial.println("");
    }

    timer = millis();
  }
}

/********************************/
/*** Initialize Functions
/********************************/

void initIMU()
{
  // Output angles in TEXT format & Turn off continuous streaming output & Disable error message output
  Serial3.write("#ot#o0#oe0");
  Serial3.flush();
}
