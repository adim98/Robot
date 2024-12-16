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
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SONAR_NUM 4     // Number of sensors.
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(8, 7, MAX_DISTANCE),
  NewPing(0, 4, MAX_DISTANCE),
  NewPing(13, 12, MAX_DISTANCE)
};

//distance variables
float distance_Rwall = 0;
float distance_Lwall = 0;

//cylinder sensor
#define trigPin1 2
#define echoPin1 1

//front wall sensor
#define trigPin2 8
#define echoPin2 7

//Left wall sensor
#define trigPin3 0
#define echoPin3 4

//right wall sensor 
#define trigPin4 13
#define echoPin4 12

//Motors 
CytronMD motor1(PWM_PWM, 3,5 );   // PWM 1A = Pin 2, PWM 1B = Pin 3.
CytronMD motor2(PWM_PWM, 6, 11); // PWM 2A = Pin 11, PWM 2B = Pin 12.

// Light Sensor variables
QTRSensors qtr;
#define Kp 0.018
#define Kd 0.0001
#define MaxSpeed 180
#define BaseSpeed 80
#define TurnSpeed 50
#define CheckPoint 800

const uint8_t SensorCount = 6;
// IR Sensors Count
uint16_t sensorValues[SensorCount];

//cylinder variable 
int collected_cyl = 0; 

void setup() 
{
  //attach the servos 
  //arm servo
  myservo.attach(10);  
  //base servo 
  myservo2.attach(9);

  //move the servos into position
  myservo.write(0);

  myservo2.write(90);

  delay(1500);
  boolean down = false;
  
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
  
  //set the servos into place 
  arm_up(); 
  arm_open();
  delay(3000);
}

void loop(){

  //start measuring wall distances 
  Serial.print("measure first time ever ");
  left_wall_cm(); 
  right_wall_cm(); 
  read_front_cm(); 

  //move through the maze 
  maze();
}

/////////////////////////////walls//////////////////////////////////////////////////////////////////////////
  //Section to measure distance of left wall
void left_wall_cm() //convert distance into cm 
{
 
  distance_Lwall = getDistance(trigPin3, echoPin3);   //variable to store the distance measured by the sensor
  Serial.print("left wall:");
  Serial.println(distance_Lwall);

   
}

//Section to measure distance of right wall
void right_wall_cm() //convert distance into cm 
{
  distance_Rwall = getDistance(trigPin4, echoPin4);
  Serial.print("right wall:");
  Serial.println(distance_Rwall);

}
//Section to measure distance in front 
void read_front_cm() 
{
  int cyl_detected = 7;              //desired cm from cylinder
  float distancefront1 = getDistance(trigPin1, echoPin1); 
  float distancefront2 = getDistance(trigPin2, echoPin2);

  if (distancefront1 == cyl_detected) 
  {
     
    if (distancefront2 > distancefront1+2)
    {
      //count number of cylinders collected 
      //collect the cylinder
      Serial.print("cylinder detected!");
      pick_up_cylinder(); 
      return;
    }
    else
    {
      Serial.print("not a cylinder: ");
      Serial.print( distancefront1);
      Serial.print("  and  ");
      Serial.println( distancefront2);
      motor_turnaround(); 
      return;
    }
 
  }
  else {
    Serial.print("distance too far: ");
    Serial.print(distancefront1);
    Serial.print(" and ");
    Serial.println(distancefront2);
    return;
  }

}
//distance sensor sensing 
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(50); // Send a pulse that is 50 ms long
  digitalWrite(trigPin, LOW);

  float echoTime = pulseIn(echoPin, HIGH);
  int distance = echoTime * 0.0344 / 2; // Calculate distance in cm
  return distance;
}
                    

////////////////////////////////////Section to pick up cylinder //////////////////////////////////////////////////
void pick_up_cylinder()
{

  Serial.println("picking up cylinder");

  delay(500);


  arm_open();

  Serial.println("op");

  delay (1500);


  arm_down();

  Serial.println("down");

  delay (1500);


  arm_close();

  Serial.println("cl");

  delay (1500);


  arm_up();

  Serial.println("up");

  delay (3000);


  arm_open();

  Serial.println("op");

  delay (1500);

  //add that you have collected a cylinder 
  collected_cyl += 1; 
  return collected_cyl; 
}

//arm movements
void arm_close()

{

  myservo2.write(60); // tell servo to go to position in variable 'pos'

  delay(15); // waits 15 ms for the servo to reach the position

}


void arm_open()

{

  myservo2.write(150); // tell servo to go to position in variable 'pos'

  delay(15);

}


void arm_down()

{

  for(int i =90; i > 0; i--){

  myservo.write(i);

  delay(25); // tell servo to go to position in variable 'pos

}

  myservo.write(0); // tell servo to go to position in variable 'pos'

  delay(150); // waits 15 ms for the servo to reach the position

}


void arm_up()

{

  myservo.write(60); // tell servo to go to position in variable 'pos' // waits 15 ms for the servo to reach the position

}

//////////////////////////////////////MOTOR/////////////////////////////////////////////////////////////////////
void motor_turnaround()
{
  motor1.setSpeed(0);  // Motor 1 stops.
  motor2.setSpeed(0);  // Motor 2 stops.
  //turn right
  motor1.setSpeed(-100);  // Motor 1 runs backward at full speed.
  motor2.setSpeed(100);   // Motor 2 runs forward at full speed.
  Serial.print("measure fl begin ");
  left_wall_cm(); 
  right_wall_cm(); 
  read_front_cm(); 
  delay (5000); //adjust to be specific amount of turning 180 degress
}

//Line follwoing 
void follow_line() {
  
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;
  float lastError = error;
  float motorSpeed = Kp * error + Kd * (error - lastError);

  left_wall_cm(); 
  right_wall_cm(); 
  read_front_cm(); 
  
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  // too Right, Robot will go left
  if (position == 5000) {
    //motor_stop();
    delay(100);
    //motor_turnright();
    delay(100);
    Serial.println("stop");

  } 
  else if (position > 3300) {
    motor1.setSpeed(BaseSpeed);
    motor2.setSpeed(BaseSpeed);
    motor_turnleft();
    return;
  } 
  else if (position < 1700) {
    motor1.setSpeed(BaseSpeed);
    motor2.setSpeed(BaseSpeed);
    motor_turnright();
    return;
  }
  else {
    motor_adjust(error);
    Serial.println("measure adjust ");
    left_wall_cm(); 
    right_wall_cm(); 
    read_front_cm();
    return;
  }



  Serial.print("Error: ");
  Serial.println(error);
 

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed + motorSpeed;

  if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
  if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  

  motor1.setSpeed(rightMotorSpeed);
  motor2.setSpeed(leftMotorSpeed);
  motor_forward();
  Serial.println("forward");
  
  Serial.print("measure fl end ");
  left_wall_cm(); 
  right_wall_cm(); 
  read_front_cm();  

}

void motor_turnleft() {
  motor1.setSpeed(TurnSpeed);   // Motor 1 runs forward at half speed.
  motor2.setSpeed(-TurnSpeed);  // Motor 2 runs backward at half speed.
  Serial.println("left");
  left_wall_cm(); 
  right_wall_cm(); 
  read_front_cm(); 
  delay(500);
}

void motor_turnright() {
  motor1.setSpeed(-TurnSpeed);  // Motor 1 runs backward at half speed.
  motor2.setSpeed(TurnSpeed);   // Motor 2 runs forward at half speed.
  Serial.println("right");
  left_wall_cm(); 
  right_wall_cm(); 
  read_front_cm(); 
  delay(500);
}
void motor_forward() {
  motor1.setSpeed(BaseSpeed);  // Motor 1 runs forward at full speed.
  motor2.setSpeed(BaseSpeed);  // Motor 2 runs forward at full speed.
  Serial.println("forward");
  delay(1000);
}
void motor_stop() {
  motor1.setSpeed(0);  // Motor 1 stops.
  motor2.setSpeed(0);  // Motor 2 stops.
  Serial.println("stop");
  delay(1000);
}
void motor_adjust(int error) {
  int adjustFactor = error/30;
  if (error<-1){
    motor1.setSpeed(BaseSpeed+adjustFactor);
  }
  else if (error>1){
    motor2.setSpeed(BaseSpeed-adjustFactor);
  }
  else {
    motor1.setSpeed(BaseSpeed);
    motor2.setSpeed(BaseSpeed);
  }
  Serial.println(adjustFactor);
  Serial.println("adjusting");
  delay(10);

}

void motor_sharpright() {
  motor1.setSpeed(30);  // Motor 1 runs backward at higher speed.
  motor2.setSpeed(75);   // Motor 2 runs forward at higher speed.
  delay(3200);            // Adjust delay for a sharper turn.
  Serial.println("sharp right");
}

void motor_sharpleft() {
  motor1.setSpeed(75);   // Motor 1 runs forward at higher speed.
  motor2.setSpeed(30);  // Motor 2 runs backward at higher speed.
  delay(3200);            // Adjust delay for a sharper turn.
  Serial.println("sharp left");
}

///////////////////////////////MAZE//////////////////////////////////////////////////////////////////////
void maze()
{
  right_wall_cm(); 
  left_wall_cm();


  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(150);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft();
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft();
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright();
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }
  
  //pick up one cylinder 

  motor_turnaround(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft();
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft();
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright();
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright();
  delay(1000);

  while (distance_Rwall <  30) 
  {
    //follow_line(); 
    motor_forward();
    delay(1000);
  }

  motor_sharpright();
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000); 

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_turnaround(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000); 

  while (distance_Lwall <  30) 
  {
    motor_forward();
    delay(1000); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_turnaround(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    motor_forward();
    delay(1000);  
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_turnaround(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_turnaround(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    motor_forward();
    delay(1000);  
  }

  motor_sharpright(); 
  delay(1000);


/////////IF THREE CYLINDERS HAVE BEEN COLLECTED GO TO EXIT////////
if (collected_cyl == 3){
  while (distance_Lwall <  30) 
  {
    motor_forward();
    delay(1000);
  }

  motor_sharpleft();
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft();
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_stop(); 
}
else { /////////IF NOT KEEP GOING ////////
  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright (); 
  delay(1000); 

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright (); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright (); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_turnaround(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_turnaround(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpright(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Lwall <  30) 
  {
    follow_line(); 
  }

  motor_sharpleft(); 
  delay(1000);

  while (distance_Rwall <  30) 
  {
    follow_line(); 
  }
  /////exit//////
  motor_stop();
}

/*forward
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

//if all three cylinders have been found exit the maze 
left 
left
exit

//if not all of the cylinders have been found keep searching and then exit 
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
exit*/
}
