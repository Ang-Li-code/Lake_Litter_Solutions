//----------------------------------
//----------------------------------
// Trouble Shooting Info
//----------------------------------
//----------------------------------

// 1) Magnetometer outputting constant value regardless of directction
/*
      Use built in functions:
      - myMag.performResetOperation();

      This should reset the magnetometer. 
      Check the keywords in the sparkfun libray if the syntax is off.
*/

//----------------------------------
//----------------------------------
// Function Conventions
//----------------------------------
//----------------------------------

// [] = Function not complete
// [O] = function complete, but not tested, "operational"
// [X] = function complete, and tested, "executable"

//----------------------------------
// General Setup
//----------------------------------

// Required Libraries
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

//-----------------------------------
// Movement Functions and Variables
//-----------------------------------

// time in all functions is how long to do each action for
// Speed tells you how fast to move the boat
// leftMotor and rightMotor, respectively, is where to connect the power pin of those motors to

// Setup needed
byte leftMotor = 8;     // Left motor pin
byte rightMotor = 10;   // Right motor pin
Servo servoL;           // Set up left motor
Servo servoR;           // Set up right motor

int moveSpeed = 150;    // Speed to move at, max is 400
int buffer = 100;       // Buffer for padding the motors
int rightBuf = 30;      // Buffer for padding the right motor
int leftBuf = 0;        // Buffer for padding the left motor
int rightForward = moveSpeed+rightBuf;
int leftForward = -(moveSpeed + leftBuf);
//------------------------------------
//------------------------------------

// Functions

void clockwiseMove(int time);         // [O] Turns clockwise
void counterclockwiseMove(int time);  // [O] Turns counterclockwise
void forwardMove(int time);           // [O] Moves forward
void backwardMove(int time);          // [O] Moves backward
void stopMove(int time);              // [O] Stop moving

//------------------------------
// GPS Functions and Variables
//------------------------------

//GPS Variables
float initLatitude = 0.0;         // Initial latitude
float initLongitude = 0.0;        // Initial longitude
int start_pinned = 0;             // [!] Use to be "flag"    
double point_boundary = 0.0001;   // [!] used to be "threshold", Define threshold for destination proximity around last waypoint.
double distance;                  // [!] used to be "dist" Define variable to hold calculated distance

// Geofence parameters
float geofenceRadius = 200.0;     // circular boundary set to 200 meter radius
bool isInsideGeofence = true;     // assuming bot is inside geofence

// Waypointer parameters
const int maxWaypoints = 5;       // Amount of waypoints allowed
int numWaypoints = 0;             // Number of way points we currently have

// Array to store latitude and longitude of waypoints
float waypoints[maxWaypoints][2];

//--------------------------------------
//--------------------------------------

// Calculation Functions

float dmsToDecimal(float dms);                                            // [X] Function to convert DMS to Decimal Degrees, this is used by Google maps as well
float calculateAngle(float latA, float lonA, float latB, float lonB);     // [O] Function to calculate angle between two GPS coordinates, lat = latitude, lon = longitude
// Must use decimal coordinates
double calculateDistance(float lat1, float lon1, float lat2, float lon2); // [O] Function to calculate distance between two GPS coordinates using haversine formula
// Must use decimal coordinates, returns value in meters
double calculateDifference(double heading, double angleToTurn)            // [O] Function to caluclate the difference between the current heading and the angle between the current location and target location.
// Interaction Functions

void printCoordinate(float coordinate, char direction);     // [X] Prints coordinate in decimal degrees
void saveInitialPosition(float latitude, float longitude);  // [O] Saves initial position
bool checkInsideGeofence(float lat, float lon);             // [O] Checks if a given coordinate is inside the geofence
double getHeading()                                         // [O] Gets the current heading from the magnetometer
double turnLeftOrRight(double difference)                   // [O] Returns 1.0 (turn right), -1.0 (turn left), or 0.0 (straight) based on the difference between the heading and calculated angle
//--------------------------------------------------------
//--------------------------------------------------------
// Main Functions
//--------------------------------------------------------
//--------------------------------------------------------

void setup()
{
  //------------------------------------------
  // Magnetometer setup
  //------------------------------------------

 Serial.println("MMC5983MA Magnetometer");

    Wire.begin();

    if (myMag.begin() == false)
    {
        Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
        while (true)
            ;
    }

    myMag.softReset();

    Serial.println("MMC5983MA connected");
}
  
  //------------------------------------------
  // GPS Set up
  //------------------------------------------

  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

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

  delay(7000); // delay to allow the ESC to recognize the stopped signal

  //--------------------------
  // Hardcoding way points
  //--------------------------

  // Hardcode additional waypoints (B, C, D, etc.) in the waypoints array as needed
  waypoints[1][0] = 30.9/* Latitude of waypoint B */;
  waypoints[1][1] = 90/* Longitude of waypoint B */;
  numWaypoints++; // Increment the number of waypoints

  waypoints[2][0] = 20 /* Latitude of waypoint C */;
  waypoints[2][1] = 40/* Longitude of waypoint C */;
  numWaypoints++; // Increment the number of waypoints
}

void loop() // Run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      // Convert GPS Coordinates from dms to decimal
      float latitude = dmsToDecimal(GPS.latitude);
      float longitude = -dmsToDecimal(GPS.longitude);

      Serial.print("Location: ");
      printCoordinate(latitude, GPS.lat);
      Serial.print(", ");
      printCoordinate(longitude, GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
  }
}

//-----------------------------------
// Movement Functions
//-----------------------------------

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

//------------------------------
// GPS Functions
//------------------------------

// Calculation Functions

float dmsToDecimal(float dms) {
  // Extract degrees, minutes, and seconds from DMS value
  int degrees = int(dms / 100);
  float minutes = (dms - (degrees * 100)) / 60.0;

  // Calculate decimal degrees
  float decimalDegrees = degrees + minutes;

  return decimalDegrees;
}

float calculateAngle(float latA, float lonA, float latB, float lonB) {
  float dLat = radians(latB - latA);
  float dLon = radians(lonB - lonA);
  latA = radians(latA);
  latB = radians(latB);
  
  float y = sin(dLon) * cos(latB);
  float x = cos(latA) * sin(latB) - sin(latA) * cos(latB) * cos(dLon);
  float angle = atan2(y, x);
  
  float angle_deg = degrees(angle);
  if (angle_deg < 0) {
    angle_deg += 360;
  }

  return angle_deg;
}

double calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const double R = 6371000.0; // Earth's radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double distance = R * c;
  return distance;
}

double calculateDifference(double heading, double angleToTurn){

  double difference =  angleToTurn-heading;
  return difference;
}

// Interaction Functions

void printCoordinate(float coordinate, char direction) {
  Serial.print(coordinate, 6); //set decimal point to 6
  Serial.print(" degrees ");
  Serial.println(direction);
}

void saveInitialPosition(float latitude, float longitude) {
  initLatitude = latitude;
  initLongitude = longitude;
  waypoints[0][0] = initLatitude;
  waypoints[0][1] = initLongitude;
  numWaypoints = 1; // The initial position is now saved as the first waypoint
  start_pinned=1;
}

bool checkInsideGeofence(float lat, float lon) {
  //Calculates distance from current point to origin point
  double distance = calculateDistance(initLatitude, initLongitude, lat, lon);
  return (distance <= geofenceRadius);
}

double getHeading(){
  
    uint32_t rawValueX = 0;
    uint32_t rawValueY = 0;
    uint32_t rawValueZ = 0;
    double scaledX = 0;
    double scaledY = 0;
    double scaledZ = 0;
    double heading = 0;

    // Read all three channels simultaneously
    myMag.getMeasurementXYZ(&rawValueX, &rawValueY, &rawValueZ);

    // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17 (131072).
    // Here we scale each field to +/- 1.0 to make it easier to calculate an approximate heading.
   
    scaledX = (double)rawValueX - 131072.0;
    scaledX /= 131072.0;

    scaledY = (double)rawValueY - 131072.0;
    scaledY /= 131072.0;

    scaledZ = (double)rawValueZ - 131072.0;
    scaledZ /= 131072.0;

    // Magnetic north is oriented with the Y axis
    // Convert the X and Y fields into heading using atan2 (Arc Tangent 2)
    heading = atan2(scaledX, 0 - scaledY);

    // atan2 returns a value between +PI and -PI
    // Convert to degrees
    heading /= PI;
    heading *= 180;
    //heading += 10;
    if(heading > 180){
      heading *= -1;
    }
    else if(heading < -180){
      heading *=-1;
    }
    return heading;
}

double turnLeftOrRight(double difference){
  double buffer = 2.5;
  //Buffer of 2.5 degrees, further testing needed
  if(difference > buffer || difference < -buffer){

      if(difference > buffer){
        return 1.0;
      }

      else if(difference < -buffer{
        return -1.0;
      }
  }
  return 0.0;
}
