//Definitions and libraries
#include <Servo.h>                                  //servo library 
#include <NewPing.h>                               //distance sensors library 
#include <QTRSensors.h>                            //include the QTR reflectance sensors library 
#include <CytronMotorDriver.h>                     // inlcudes motor library 

// create Servo object to control a servo
 //base servo 
Servo myservo; 
//arm servo 
Servo myservo2; 

//Distance Sensors 
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
float distance = 0; 
//cylinder sensor
#define trigPin1 5
#define echoPin1 4

//front wall sensor
#define trigPin2 0
#define echoPin2 1

//Left wall sensor
#define trigPin3 13
#define echoPin3 12

//right wall sensor 
#define trigPin4 8
#define echoPin4 7

//Motors 
CytronMD motor1(PWM_PWM, 2, 3);   // PWM 1A = Pin 2, PWM 1B = Pin 3.
CytronMD motor2(PWM_PWM, 11, 12); // PWM 2A = Pin 11, PWM 2B = Pin 12.

// Light Sensor variables
QTRSensors qtr;
#define Kp 0.018
#define Kd 0.0001
#define MaxSpeed 180
#define BaseSpeed 55

const uint8_t SensorCount = 6;
// IR Sensors Count
uint16_t sensorValues[SensorCount];

//cylinder variable 
const int collected_cyl = 0; 

void setup() 
{
  //attach the servos 
  //arm servo
  myservo.attach(10);  
  //base servo 
  myservo2.attach(9);
  
  //establish wall sensor setup
  pinMode(trigPin1, OUTPUT);                         //the trigger pin will output pulses of electricity
  pinMode(echoPin1, INPUT);                          //the echo pin will measure the duration of pulses coming back from the distance sensor
  pinMode(trigPin2, OUTPUT);                         //the trigger pin will output pulses of electricity
  pinMode(echoPin2, INPUT);                          //the echo pin will measure the duration of pulses coming back from the distance sensor
  pinMode(trigPin3, OUTPUT);                         //the trigger pin will output pulses of electricity
  pinMode(echoPin3, INPUT);                          //the echo pin will measure the duration of pulses coming back from the distance sensor
  pinMode(trigPin4, OUTPUT);                         //the trigger pin will output pulses of electricity
  pinMode(echoPin4, INPUT);                          //the echo pin will measure the duration of pulses coming back from the distance sensor
  
  //establish line sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, SensorCount);

  Serial.begin(9600);
  delay(500);
  Serial.println("QTR calibration");

  pinMode(LED_BUILTIN, OUTPUT);

  // LED turns ON to indicate the start of the calibration
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  //callibrate the line sensor
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    Serial.println(i);
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration
  
  delay(3000);

}

void loop() 
{
  //set the servos into place 
  arm_up(); 
  arm_open();

  //start measuring wall distances 
  left_wall_cm(); 
  right_wall_cm(); 
  read_front_cm(); 

  //move through the maze 
  maze();

}


//Section to measure distance of left wall
void left_wall_cm() //convert distance into cm 
{
  SonarSensor(trigPin3, echoPin3);
  int Lwall_distance = 5;            //desired cm from wall 
  float distance_Lwall = distance;   //variable to store the distance measured by the sensor
  delay (500);                     //wait 500 ms between ping 
}

//Section to measure distance of right wall
void right_wall_cm() //convert distance into cm 
{
  int Rwall_distance = 5;            //desired cm from wall
  SonarSensor(trigPin4, echoPin4);
  float distance_Rwall = distance;
  delay (500);                     //wait 500 ms between ping 
}

//Section to measure distance in front 
void read_front_cm() //convert distance into cm 
{
  int cyl_detected = 7;              //desired cm from cylinder
  SonarSensor(trigPin3, echoPin3);
  float distancefront1 = distance;

  delay (500);                     //wait 500 ms between ping 

  if (distancefront1 == cyl_detected) 
  {
  SonarSensor(trigPin4, echoPin4);
  float distancefront2 = distance;
  delay (500);                     //wait 500 ms between ping 
    if (distancefront2 > distancefront1+2)
    {
      //count number of cylinders collected 
      //collect the cylinder
      pick_up_cylinder(); 
      return;
    }
    else
    {
      Serial.print("not a cylinder:");
      Serial.print(distancefront1);
      Serial.print(" and ");
      Serial.print(distancefront2);
      turn_around(); 
      return;
    }
 
  }
  else {
    Serial.print("distance too far:");
    Serial.println(distancefront1);
    return;
  }

}

void SonarSensor(int trigPin,int echoPin)
{
  //send out an ultrasonic pulse that's 50ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin, LOW);

  float echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the pulse to bounce back to the sensor
  distance = sonar.ping_cm();   //variable to store the distance measured by the sensor
  return distance; 
  delay (500);                     //wait 500 ms between ping 

}

//Section to pick up cylinder 
void pick_up_cylinder()
{
  Serial.println("picking up cylinder");
  delay(500); 
  
  arm_open();
  Serial.println("open"); 
  delay (1500);

  arm_down(); 
  Serial.println("down");
  delay (1500);

  arm_close(); 
  Serial.println("close");
  delay (1500);

  arm_up(); 
  Serial.println("up");
  delay (3000);

  arm_open();
  Serial.println("open"); 
  delay (1500);
    return;  

}
//arm movements 
void arm_close()
{
  myservo.write(0);              // tell servo to go to position in variable 'pos'
  delay(15);                       // waits 15 ms for the servo to reach the position
}

void arm_open()
{
  myservo.write(75);              // tell servo to go to position in variable 'pos'
  delay(15); 
}

void arm_down()
{
  myservo2.write(0);              // tell servo to go to position in variable 'pos'
  delay(150);               // waits 15 ms for the servo to reach the position
}

void arm_up()
{
  myservo2.write(90);              // tell servo to go to position in variable 'pos'
  delay(150);                       // waits 15 ms for the servo to reach the position
}

void turn_around()
{
  motor1.setSpeed(0);  // Motor 1 stops.
  motor2.setSpeed(0);  // Motor 2 stops.
  //turn right
  motor1.setSpeed(-100);  // Motor 1 runs backward at full speed.
  motor2.setSpeed(100);   // Motor 2 runs forward at full speed.
  delay (50000); //adjust to be specific amount of turning 180 degress
}

//maze 
void maze()
{
forward
sharpright
forward
junction-left
forward
junction-left
forward
right
forard 
cylinder 
turnaround 
forward
left
forward 
junction - left
forward
right
foward
right 
forward
right
forward
left 
foward
junction - left 
foward
turn around 
foward
left 
forward 
left 
foward 
right 
forward 
turnaround 
forward
left 
forward 
left 
forward 
right 
forward 
right 
forward 
right 
forward 
right 
forward
turnaround 
forward 
right 
forward
left 
forward
turnaround 
forward 
right 
foward
right 
forward 

//if all three cylinders have been found 
left 
left
exit

//if not all of the cylinders have been found 
right 
forward
right 
forward
right 
forward
right 
forward
left 
forward
left 
forward 
right 
forward
right 
forward
turnaround 
forward
left
forward
left
forward
left 
forward
right 
forward
left
forward
left
forward
turnaround
forward
right 
forward
right 
forward
left
forward
left 
forward
right
forward
right 
forward
left 
left
exit
}
