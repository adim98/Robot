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
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp, x;
int lfspeed = 400;

float Kp = 0.1;  // Adjust this value based on experimentation
float Kd = 0.1;  // Adjust this value based on experimentation
float Ki = 0.01; // Adjust this value based on experimentation


void setup() 
{
  Serial.begin (9600);                              //set up a serial connection with the computer
  Serial.println("QTR calibration");

  //establish line sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  //qtr.setSensorPins((const uint8_t[]){sensorRR, sensorR, sensorRM, sensorLM, sensorL, sensorLL}, sensorCount);
  qtr.setEmitterPin(13);
  
  // Calibration
  delay(500);
  Serial.println("calibrating");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  //callibrate the line sensor
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    Serial.println(i); 
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  //Motor and sensor pin setup 
  //pinMode(sensorRR,INPUT);                           // the pin will measure the most right line sensor value 
  //pinMode(sensorR,INPUT);                           // the pin will measure the right line sensor value 
  //pinMode(sensorRM,INPUT);                           // the pin will measure the right middle line sensor value 
  //pinMode(sensorLM,INPUT);                           // the pin will measure the left middle line sensor value 
  //pinMode(sensorL,INPUT);                           // the pin will measure the left line sensor value 
  //pinMode(sensorLL,INPUT);                           // the pin will measure the most left line sensor value 
  

  //myservo.attach(3);                              //use pin 3 to control the servo


}

void loop() 
{

  float desired_position = 2800;                              // keep line in the center of the sensor 
  float actual_position = qtr.readLineBlack(sensorValues);     //reads the acutal position  
  float error = (desired_position)-(actual_position);         // how far away from the line is the robot 
  float turn_signal = Kp*error;                               // how much should the robot turn 
  
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position


  if (turn_signal >= 50)
  {
    Serial.println(turn_signal);
    motor_turnleft();
      for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(actual_position);

    delay(25);
    motor_forward();
  } 
  else if (turn_signal <= 100)
  {
    Serial.print(turn_signal);
    motor_turnright();
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println(actual_position);

      delay(250);
    motor_forward();

  }
  else if (turn_signal == 0)
  {
    Serial.print(turn_signal);
    motor_forward();
    for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println(actual_position);

      delay(250);
  }
  else 
  {
    Serial.print(turn_signal);
    motor_stop();
    for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println(actual_position);

      delay(250);
  }


}

void motor_turnleft()
{
  motor1.setSpeed(255);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-255);  // Motor 2 runs backward at full speed.
  delay(1000);
}

void motor_turnright ()
{
  motor1.setSpeed(-255);   // Motor 1 runs backward at full speed.
  motor2.setSpeed(255);  // Motor 2 runs forward at full speed.
  delay(1000);
}
void motor_forward()
{
  motor1.setSpeed(255);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(255);  // Motor 2 runs forward at full speed.
  delay(1000);
}
void motor_stop()
{
  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(1000);
}

