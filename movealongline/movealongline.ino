#include <CytronMotorDriver.h>                     // inlcudes motor library
#include <QTRSensors.h>
QTRSensors qtr;


// Configure the motor driver
/* CONNECTIONS:
 * Arduino D6  - Motor Driver PWM 1A Input
 * Arduino D10  - Motor Driver PWM 1B Input
 * Arduino D5 - Motor Driver PWM 2A Input
 * Arduino D8 - Motor Driver PWM 2B Input
 * Arduino GND - Motor Driver GND */
CytronMD motor1(PWM_PWM, 6, 10);   // PWM 1A = Pin 6, PWM 1B = Pin 10.
CytronMD motor2(PWM_PWM, 5, 9); // PWM 2A = Pin 5, PWM 2B = Pin 8.

#define Kp 0.018
#define Kd 0.0001
#define MaxSpeed 180
#define BaseSpeed 55
#define SpeedTurn 15
#define CheckPoint 800

const uint8_t SensorCount = 6;
// IR Sensors Count 
uint16_t sensorValues[SensorCount];

void setup()
{   
    //establish line sensor
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

    Serial.begin(9600);
    delay(500);
    Serial.println("QTR calibration");

    
    pinMode(LED_BUILTIN, OUTPUT);

    // LED turns ON to indicate the start of the calibration
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    //callibrate the line sensor
    for (uint16_t i = 0; i < 400; i++)
    {
      qtr.calibrate();
      Serial.println(i); 
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

    delay(3000); 
}  

void loop()
{  
  followLine();
}

void followLine(){

    uint16_t position = qtr.readLineBlack(sensorValues);  
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(position);

    // too Right, Robot will go left
    if(position == 5000) 
    {
      //motor_stop(); 
      delay(100); 
      //motor_turnright();
      delay(100);
      Serial.println("stop");

    }
    else if (position>2800)  
    {
        motor_turnright(); 
        return;    
    }
    else if(position<2200)
    {  
        motor_turnleft();
        return;
    }
    

    float error = position - 2500;
    float lastError = error;
    float motorSpeed = Kp * error + Kd * (error - lastError);
    

    Serial.print("Error: ");
    Serial.println(error);

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
  
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; 
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    
    motor1.setSpeed(rightMotorSpeed);
    motor2.setSpeed(leftMotorSpeed);
    motor_forward();
    Serial.println("forward");
}

void motor_turnleft()
{
  motor1.setSpeed(5);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-5);  // Motor 2 runs backward at full speed.
  delay(1000);
  
}

void motor_turnright ()
{
  motor1.setSpeed(-5);   // Motor 1 runs backward at full speed.
  motor2.setSpeed(5);  // Motor 2 runs forward at full speed.
  delay(1000);
  
}
void motor_forward()
{
  motor1.setSpeed(100);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(100);  // Motor 2 runs forward at full speed.
  Serial.println("forward");
  delay(1000);
}
void motor_stop()
{
  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  Serial.println("stop");
  delay(1000);
}
