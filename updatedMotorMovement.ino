//Compute magnetic heading from the MMC5983MA

//Required Libraries
#include <Wire.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
#include <Servo.h>

//SFE_MMC5983MA myMag;                            //Magnetometer name
Servo servoR;           // Set up left motor
Servo servoL;           // Set up right motor
byte leftMotor = 8;     // Left motor pin
byte rightMotor = 10;   // Right motor pin


int moveSpeed = 125;
int buffer = 100;

int rightBuf = 30;
int leftBuf = 0;

int rightForward = moveSpeed+rightBuf;
int leftForward = -(moveSpeed + leftBuf);

void setup()
{

    //-----------------------------
    // Magnetometer Set up
    //-----------------------------

    Serial.begin(115200);                       //Set baud rate to 115200
    Serial.println("MMC5983MA Magnetometer");

   // Wire.begin();

    
    //-----------------------------
    // Motor Set up
    //-----------------------------

    //Setting up the output pins to left and right motor, respectively
    servoL.attach(leftMotor);
    servoR.attach(rightMotor);

    servoL.writeMicroseconds(1500); // send "stop" signal to ESC.
    servoR.writeMicroseconds(1500);
    //Maximum foward movement: 1900
    //Maximum backward movement: 1100
    //Stop: 1500
    //Speed depends on how close you are to the maximum

    //delay(7000); // delay to allow the ESC to recognize the stopped signal
    delay(15000);


}

void loop()
{
forwardMove(8500);
delay(8000);
}

void clockwiseMove(int time){
  //Move left motor forward and right motor backward
  servoR.writeMicroseconds(1500 - rightForward);
  servoL.writeMicroseconds(1500 + leftForward);

  //Time to do this for
  delay(time);
  stopMove();
}

void counterClockwiseMove(int time){
  //Move right motor forward and left motor backward
  servoL.writeMicroseconds(1500 - leftForward);
  servoR.writeMicroseconds(1500 + rightForward);
  
  //Time to do this for
  delay(time);
  stopMove();
}

void forwardMove(int time){
  //Move the motors forward
  servoL.writeMicroseconds(1500 + rightForward);
  servoR.writeMicroseconds(1500 + leftForward);
  
  //Time to do this for
  delay(time);
  stopMove();
}

void backwardMove(int time){
  //Move the motors backward
  servoL.writeMicroseconds(1500 - rightForward);
  servoR.writeMicroseconds(1500 - leftForward);
  
  //Time to do this for
  delay(time);
  stopMove();
}

void stopMove(){
  //Stop both motors
  servoR.writeMicroseconds(1500);
  servoL.writeMicroseconds(1500);
}
