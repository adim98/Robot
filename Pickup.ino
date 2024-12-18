/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo clawservo;  // create Servo object to control a servo
Servo armservo;
// twelve Servo objects can be created on most boards

int up = 180, down = 90, closed = 80, open = 180;    // variable to store the servo position

void setup() {
  clawservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  armservo.attach(10);
}

void loop() {
  armservo.write(up);
  clawservo.write(open);
  delay(1000);

  for(int i = up; i >= down; i--) {
    armservo.write(i);
    delay(5);
  }


  delay(200);

  clawservo.write(closed);
  delay(200);
  float upDelay = 50;
  for(int i = down; i <= up; i++) {
    armservo.write(i);
    delay(upDelay);
    upDelay = upDelay*0.9;
  }
  delay(300);
  clawservo.write(open);

  delay(5000);

  /*
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  */
}
