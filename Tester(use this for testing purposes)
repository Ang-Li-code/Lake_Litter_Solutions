#include <Servo.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port | very important we use hardware port on ATmega 
Adafruit_GPS GPS(&GPSSerial); 

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console - from adafruit
// Set to 'true' if you want to debug and listen to the raw GPS sentences - adafruit
#define GPSECHO false

uint32_t timer = millis();

//sensehat
LSM9DS1 imu; //this is the magnetometer module used inside the sense hat.

//these setup the pins used to communicate with sense hat thru I2C
#define LSM9DS1_M 0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6A // Would be 0x6A if SDO_AG is LOW   

//Set up left motor and right motor, respectively, to their appropriate pins
byte leftMotor = 8;
byte rightMotor = 10;
Servo servoL;
Servo servoR;

//Speed to move at, max is 400
int moveSpeed = 100;

//initializing some vars
float initLatitude = 0.0; //Initial latitude
float initLongitude = 0.0; //Initial longitude
int flag = 0;
double threshold = 0.0001; // Define threshold for destination proximity around last waypoint.
double dist; // Define variable to hold calculated distance

// Geofence parameters
float geofenceRadius = 200.0; // 200 meters
bool isInsideGeofence = true;

//float BLat = 33.418379;
//float BLong = 111.932695;
// Function to save the initial position (A)
//Hardcoded initial position - coords are for noble library

//this function uses the magnetometer to establish an angle, calculates what angle you need to be from north to get from point A to B
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

//following functions are various motor controll functions

void clockwiseMove(){
  //Move left motor forward and right motor backward
  servoL.writeMicroseconds(1500 + moveSpeed);
  servoR.writeMicroseconds(1500 - moveSpeed);
}

void counterclockwiseMove(){
  //Move right motor forward and left motor backward
  servoR.writeMicroseconds(1500 + moveSpeed);
  servoL.writeMicroseconds(1500 - moveSpeed);
}

void forwardMove(){
  //Move the motors forward
  servoL.writeMicroseconds(1500 + moveSpeed);
  servoR.writeMicroseconds(1500 + moveSpeed);
}

void backwardMove(){
  //Move the motors backward
  servoL.writeMicroseconds(1500 - moveSpeed);
  servoR.writeMicroseconds(1500 - moveSpeed);
}

void stopMove(){
  //Stop both motors
  servoR.writeMicroseconds(1500);
  servoL.writeMicroseconds(1500);
}

// Print coordinate in decimal degrees
void printCoordinate(float coordinate, char direction) {
  Serial.print(coordinate, 6); //set decimal point to 6
  Serial.print(" degrees ");
  Serial.println(direction);
}

//function to convert DMS to Decimal Degrees 
float dmsToDecimal(float dms) {
  // Extract degrees, minutes, and seconds from DMS value
  int degrees = int(dms / 100);
  float minutes = (dms - (degrees * 100)) / 60.0;

  // Calculate decimal degrees
  float decimalDegrees = degrees + minutes;

  return decimalDegrees;
}

// Function to calculate the distance between two GPS coordinates using Haversine formula
// Returns the distance in meters
double calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const double R = 6371000.0; // Earth's radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double distance = R * c;
  return distance;
}

//Function to check if a given coordinate is inside the geofence
bool checkInsideGeofence(float lat, float lon) {
  //Calculates distance from current point to origin point
  double distance = calculateDistance(initLatitude, initLongitude, lat, lon);
  return (distance <= geofenceRadius);
}

// this var controls maximum number of waypoints - adjusting this adjusts array size 
const int maxWaypoints = 5;
int numWaypoints = 0;

// Array to store latitude and longitude of waypoints
float waypoints[maxWaypoints][2];

//Saves initial position
void saveInitialPosition(float latitude, float longitude) {
  initLatitude = latitude;
  initLongitude = longitude;
  waypoints[0][0] = initLatitude;
  waypoints[0][1] = initLongitude;
  numWaypoints = 1; // The initial position is now saved as the first waypoint
  flag=1;
}

// Function to navigate to the next waypoint in the list - movement function!
void navigateToNextWaypoint() {
  if (numWaypoints == 0) {
    Serial.println("No waypoints set.");
    return;
  }

  //Goes from one way point to the next
  for (int i = 0; i < numWaypoints - 1; i++) { // Start from index 0 and go until the second-to-last waypoint
    // Check if the waypoint is inside the geofence
    if (!checkInsideGeofence( waypoints[i + 1][0], waypoints[i + 1][1])) {
      Serial.println("Waypoint outside the geofence. Cannot proceed.");//this code prevents the robot from moving once outside the geofence. this will need to be updated to allow robot to move back into boundary.
      return;
    }

    //Finds the angle to turn
    float angleToTurn = calculateAngle(waypoints[i][0], waypoints[i][1], waypoints[i + 1][0], waypoints[i + 1][1]);//calculate the angle
    Serial.print("Angle to Turn: ");
    Serial.println(angleToTurn);

    // Navigation code (similar to your existing code)
    if (imu.magAvailable()){
      //Reads where the bot is headed
      imu.readMag();
      float heading_rad = atan2(imu.my, imu.mx); //Uses arctangent
      float heading_deg = degrees(heading_rad); 

      // atan2 returns a result in the range -PI to PI. Convert to 0 to 2PI.
      if (heading_deg < 0) 
      {
        heading_deg += 360;
      } 

      Serial.print("Heading: ");
      Serial.println(heading_deg);
    
      //Check that the robot faces the correct way before it starts to move
      while( !((heading_deg<(angleToTurn+2)) && (heading_deg>(angleToTurn-2))) ) 
      {
        float difference = angleToTurn - heading_deg;
        if ((difference > 0 && difference <= 180) || (difference < -180)) {
          clockwiseMove(); // if difference is between 0 and 180 or less than -180, turn clockwise
          delay(1000);
          Serial.print("Turning clockwise");
        } else {
          counterclockwiseMove(); // otherwise, turn counterclockwise
          delay(1000);
          Serial.print("Turning counterclockwise");
        }
        imu.readMag();
        heading_rad = atan2(imu.my, imu.mx);
        heading_deg = degrees(heading_rad);

        //Changing degrees to 2PI
        if (heading_deg < 0) {
          heading_deg += 360;
        }
        Serial.print("Heading: ");
        Serial.println(heading_deg);
      }
    
      do {
        unsigned long startTime = millis(); // Record the start time

        //Move forward for 10000 milisecond 
        while((millis() - startTime) < 10000)
        { 
          forwardMove();
        }

        Serial.print("forward");
        delay(2000); // delay dynamic based on distance to target

        if (GPS.fix) //Checks if the GPS is working, may need to check the data later
        {
          //Update variables
          float latitude = dmsToDecimal(GPS.latitude);
          float longitude = dmsToDecimal(GPS.longitude);
          printCoordinate(latitude, GPS.lat);
          printCoordinate(longitude, GPS.lon);

          // Check if the robot is inside the geofence before moving to the next waypoint
          if (!checkInsideGeofence(latitude, longitude)) {
            Serial.println("Outside the geofence. Cannot proceed.");
           return;
          }
          // Euclidean distance between the current and the target point
          //euclDistance = sqrt(pow(latitude - waypoints[i + 1][0], 2) + pow(longitude - waypoints[i + 1][1], 2));

          //Update variables
          float dist = calculateDistance(latitude, longitude, waypoints[i + 1][0], waypoints[i + 1][1]);
          float angleToTurn = calculateAngle(latitude, longitude, waypoints[i + 1][0], waypoints[i + 1][1]);
          Serial.print("Angle to Turn: ");
          Serial.println(angleToTurn);

          //Check if the magnitometer is working, and then adjust the angle of the robot
          if (imu.magAvailable())
          { //we want to calculate the angle again from where the robot is currently facing to the target
            imu.readMag();
            float heading_rad = atan2(imu.my, imu.mx);
            float heading_deg = degrees(heading_rad);

            // atan2 returns a result in the range -PI to PI. Convert to 0 to 2PI.
            if (heading_deg < 0) {
              heading_deg += 360;
            }

            // Adjust the robot's heading again to match the desired angle
            while( !((heading_deg<(angleToTurn+2)) && (heading_deg>(angleToTurn-2))) ) 
            {
              float difference = angleToTurn - heading_deg;
              if ((difference > 0 && difference <= 180) || (difference < -180)) {
                clockwiseMove(); // if difference is between 0 and 180 or less than -180, turn clockwise
                delay(1000);
                Serial.print("Turning clockwise");
              } else {
                counterclockwiseMove();// otherwise, turn counterclockwise
                delay(1000);
                Serial.print("Turning counterclockwise");
              }
              imu.readMag();
              heading_rad = atan2(imu.my, imu.mx);
              heading_deg = degrees(heading_rad);
              if (heading_deg < 0) {
                heading_deg += 360;
              }
              Serial.print("Heading: ");
              Serial.println(heading_deg);
            }
          } 
        } else {
          exit(0);
        }
      } while (dist > threshold);
    }
    // Stop the robot briefly before moving to the next waypoint
    stopMove();
    delay(2000); // Adjust this delay as needed
  }

  // Reset the number of waypoints for future use
  numWaypoints = 0;
}
 
void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  //uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude//-----RMC AND GGA ADDITION!!!------
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
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

  //Serial.begin(115200); //sensehat

  Wire.begin();

  if (!imu.begin(LSM9DS1_AG, LSM9DS1_M))
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }

  // Hardcode additional waypoints (B, C, D, etc.) in the waypoints array as needed
  waypoints[1][0] = 30.9/* Latitude of waypoint B */;
  waypoints[1][1] = 90/* Longitude of waypoint B */;
  numWaypoints++; // Increment the number of waypoints

  waypoints[2][0] = 20 /* Latitude of waypoint C */;
  waypoints[2][1] = 40/* Longitude of waypoint C */;
  numWaypoints++; // Increment the number of waypoints

  // Add more waypoints as needed following the same pattern above
}

void loop() // run over and over again
{
  int signal = 1550; // Set signal value, which should be between 1100 and 1900
  servoL.writeMicroseconds(signal); // Send signal to right ESC
  servoR.writeMicroseconds(signal);

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

  if (GPS.fix) {
    Serial.print("Location: ");
    float latitude = dmsToDecimal(GPS.latitude);
    printCoordinate(latitude, GPS.lat);
    float longitude = dmsToDecimal(GPS.longitude);
    printCoordinate(longitude, GPS.lon);

    // Check if the initial position (A) is not set
    if(flag==0) {
      // Save the initial position (A) if not set
      saveInitialPosition(latitude, longitude);
      Serial.print("x");
    }
     
    else{

      // Update the geofence status based on current GPS position
      isInsideGeofence = checkInsideGeofence(latitude, longitude);
      // If the robot is outside the geofence, stop moving
      if (!isInsideGeofence) {
        exit(0);
      }
      // Check for a trigger to start navigating to waypoints (you can adjust this condition as needed)
      //if (/* Add your trigger condition here, e.g., a button press or specific GPS position */) {
        navigateToNextWaypoint();
      //}
    }
  }
}

