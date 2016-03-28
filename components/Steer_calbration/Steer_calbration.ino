#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 85;    // variable to store the servo position

void setup() {
   Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
   myservo.attach(8);  // attaches the servo on pin 9 to the servo object

}

void loop() {
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(1000);                       // waits 15ms for the servo to reach the position
}
