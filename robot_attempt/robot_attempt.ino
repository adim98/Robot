//code to control robot 


//Definitions and libraries
#include <QTRSensors.h>                            //include the QTR reflectance sensors library 
#include <Servo.h>                                 //include the servo library
#include <CytronMotorDriver.h>                     // inlcudes motor library 

const int trigPin = 6;                             //connects to the trigger pin on the wall distance sensor
const int echoPin = 7;                             //connects to the echo pin on the wall distance sensor

float distancewall = 0;                            //stores the distance from wall measured by the wall sensor

//Servo myservo;                                     //create a servo object

// Configure the motor driver
/* CONNECTIONS:
 * Arduino D6  - Motor Driver PWM 1A Input
 * Arduino D10  - Motor Driver PWM 1B Input
 * Arduino D5 - Motor Driver PWM 2A Input
 * Arduino D8 - Motor Driver PWM 2B Input
 * Arduino GND - Motor Driver GND */
CytronMD motor1(PWM_PWM, 6, 10);   // PWM 1A = Pin 6, PWM 1B = Pin 10.
CytronMD motor2(PWM_PWM, 5, 9); // PWM 2A = Pin 5, PWM 2B = Pin 8.

// Sensor pins
#define sensorRR A0
#define sensorR A1
#define sensorRM A2
#define sensorLM A3
#define sensorL A4
#define sensorLL A5

QTRSensors qtr;
const int sensorCount = 6;
const int sensorPins[] = {A0, A1, A2, A3, A4, A5};
uint16_t sensorValues[sensorCount];

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp, x;
int lfspeed = 400;

float Kp = 1.6;  // Adjust this value based on experimentation
float Kd = 0.1;  // Adjust this value based on experimentation
float Ki = 0.01; // Adjust this value based on experimentation


//setup function 
void setup() 
{
  Serial.begin (9600);                              //set up a serial connection with the computer
  

  //establish line sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){sensorRR, sensorR, sensorRM, sensorLM, sensorL, sensorLL}, sensorCount);
  qtr.setEmitterPin(13);
  
  // Calibration
  delay(500);
  Serial.println("calibrating");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  //Motor and sensor pin setup 
  pinMode(sensorRR,INPUT);                           // the pin will measure the most right line sensor value 
  pinMode(sensorR,INPUT);                           // the pin will measure the right line sensor value 
  pinMode(sensorRM,INPUT);                           // the pin will measure the right middle line sensor value 
  pinMode(sensorLM,INPUT);                           // the pin will measure the left middle line sensor value 
  pinMode(sensorL,INPUT);                           // the pin will measure the left line sensor value 
  pinMode(sensorLL,INPUT);                           // the pin will measure the most left line sensor value 
  
  pinMode(engineL,OUTPUT);                           // the pin will send commands to the left motor 
  pinMode(inputL1,OUTPUT);                           // the pin will send commands to the left motor to stop 
  
  pinMode(engineR,OUTPUT);                           // the pin will send commands to the right motor  
  pinMode(inputR1,OUTPUT);                           // the pin will send commands to the left motor 
  //establish wall sensor setup
  pinMode(trigPin, OUTPUT);                         //the trigger pin will output pulses of electricity
  pinMode(echoPin, INPUT);                          //the echo pin will measure the duration of pulses coming back from the distance sensor

  
  //myservo.attach(3);                              //use pin 3 to control the servo

}

//main code to run 
void loop() 
{

  //myservo.writeMicroseconds(1500)                 //sets the desired position for a servo 
  linefollow();   
  read_sonar_cm();                                   
  motor_run(); 
  


}

// Section to measure distance from line
void linefollow() 
{

  float desired_position = 2800;                              // keep line in the center of the sensor 
  float actual_position = qt.readLineBlack(sensorValues);     //reads the acutal position 
  const kp = 0.1; 
  float error = (desired_position)-(actual_position);         // how far away from the line is the robot 
  float turn_signal = kp*error;                               // how much should the robot turn 

  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print('\t');

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    if (sensorValues[i] > sensorThreshold[i]) 
    {
      Serial.print("X");  // black surface detected
    }
    else 
    {
      Serial.print("-");  // white surface detected
    }
  }
  Serial.println();
  delay(250);
}

//Section to measure diastance from wall 
float read_sonar_cm() //convert distance into cm 
{
  int desired_wall = 5 //cm from wall 
  float actual_wall = read_sonar_cm()
  float error = desired_wall - actual_wall; 
  const kpw = 0.1; 
  float turn_signal = kp*error
  float echoTime;                               //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;                     //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);            //use the pulsein command to see how long it takes for the pulse to bounce back to the sensor
  calculatedDistance = echoTime / 58.773;       //calculate the distance of the object in cm that reflected the pulse (half the bounce time multiplied by the speed of sound)
  return calculatedDistance;                    //send back the distance that was calculated
}

//Section to get motor to move 
void motor_run()
{
  motor1.setSpeed(128);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(-128);  // Motor 2 runs backward at 50% speed.
  delay(1000);
  
  motor1.setSpeed(255);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-255);  // Motor 2 runs backward at full speed.
  delay(1000);

  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(1000);

  motor1.setSpeed(-128);  // Motor 1 runs backward at 50% speed.
  motor2.setSpeed(128);   // Motor 2 runs forward at 50% speed.
  delay(1000);
  
  motor1.setSpeed(-255);  // Motor 1 runs backward at full speed.
  motor2.setSpeed(255);   // Motor 2 runs forward at full speed.
  delay(1000);

  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(1000);
}

//Section to detect cylinder 

//Section to pick up cylinder 