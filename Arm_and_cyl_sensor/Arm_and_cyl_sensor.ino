/*
Servo arm code and distance sensor
*/
#include <Servo.h>                //include the servo library
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int trigPin = 6;           //connects to the trigger pin on the distance sensor
const int echoPin = 7;           //connects to the echo pin on the distance sensor

float distancecyl = 0;           //stores the distance measured by the distance sensor looking for cyliners

Servo myservo;                    //create a servo object

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();
//define servos and pins
#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3


void setup()
{
  Serial.begin (9600);        //set up a serial connection with the computer

  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity
  pinMode(echoPin, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor

  // establish connection to servos
  srituhobby.begin();
  srituhobby.setPWMFreq(60);
  srituhobby.setPWM(servo1, 0, 330);
  srituhobby.setPWM(servo2, 0, 150);
  srituhobby.setPWM(servo3, 0, 300);
  srituhobby.setPWM(servo4, 0, 410);
  delay(3000);
}
void loop() {
  distancecyl = getDistance();   //variable to store the distance measured by the sensor

  if (distancecyl <= 10) {                       //if the object is close
    
    servo_positions(); 
    delay(100);                   //wait 100 milliseconds
  } 
  else 
  {
  Serial.print(distancecyl);     //print the distance that was measured
  Serial.println(" cm ");      //print units after the distance
  }
  delay(50);      //delay 50ms between each reading
}

//------------------FUNCTIONS-------------------------------

//RETURNS THE DISTANCE MEASURED BY THE DISTANCE SENSOR
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the pulse to bounce back to the sensor

  calculatedDistance = (echoTime * 0.034)/2 ;  //calculate the distance of the object that reflected the pulse 

  return calculatedDistance;              //send back the distance that was calculated
}

//arm grabs cylinders 0 degree = 150 and 180 degree = 600
void servo_positions()
{
  for (int S1value = 330; S1value >= 250; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }

  for (int S2value = 150; S2value <= 380; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 300; S3value <= 380; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 410; S4value <= 510; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ////////////////////////
  delay(2000);
  for (int S4value = 510; S4value > 410; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S3value = 380; S3value > 300; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 380; S2value > 150; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 250; S1value < 450; S1value++) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
  //////////////////////
  for (int S2value = 150; S2value <= 380; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 300; S3value <= 380; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 410; S4value <= 510; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S4value = 510; S4value > 410; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ///////////////////
  for (int S3value = 380; S3value > 300; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 380; S2value > 150; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 450; S1value > 330; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
}