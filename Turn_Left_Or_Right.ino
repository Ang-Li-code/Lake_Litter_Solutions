//Compute magnetic heading from the MMC5983MA
//Distance from ECG to Taco Bell is a good test example
//Required Libraries
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA

SFE_MMC5983MA myMag;                            //Magnetometer name

void setup()
{
    Serial.begin(115200);                       //Set baud rate to 115200
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

void loop()
{
   //double latA = dmsToDecimal();
   //double longA = dmsToDecimal(5654.6);
  // double latB = dmsToDecimal();
  //double longB = dmsToDecimal();
    double currentHeading = getHeading();
    double currentAngleToTurn = calculateAngle(33.42012,-111.93176,33.42025,-111.93176);
    double difference = getDifference(currentHeading, currentAngleToTurn);
    difference = difference - 15;
    
    if(difference < -180){
      difference += 360;
    }
   
    String leftOrRight = turnLeftOrRight(difference);
    
    Serial.print("Angle: ");
    Serial.print(currentAngleToTurn);
    Serial.print("  difference: ");
    Serial.print(difference);
    Serial.print("  direction to Turn: ");
    Serial.println(leftOrRight);
    delay(300);
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



double calculateAngle(double latA, double lonA, double latB, double lonB) {
 
  double dy = sin(radians(lonB-lonA))  * cos(radians(latB));

  double dx = cos(radians(latA)) * sin(radians(latB)) - sin(radians(latA)) * cos(radians(latB)) * cos(radians(lonB-lonA));
  
  double angle = atan2(dy,dx);

  angle = degrees(angle);
  
  //If x is negative and y is positive, 
  //Compensate field of detection
 

  //angle_deg = 90 - angle_deg;

   //Removed padding           

  return angle;
}

double getDifference(double heading, double angleToTurn){

  double difference =  angleToTurn-heading;
 // if(difference < 0)
  //difference += 360;
  return difference;
}



double dmsToDecimal(double dms) {
  // Extract degrees, minutes, and seconds from DMS value
  int degrees = int(dms / 100);
  double minutes = (dms - (degrees * 100)) / 60.0;

  // Calculate decimal degrees
  double decimalDegrees = degrees + minutes;

  return decimalDegrees;
}

String turnLeftOrRight(double difference){
  
  if(difference > 2.5 || difference < -2.5){

      if(difference > 2.5){
        return "Turn right";
      }

      else if(difference < -2.5){
        return "Turn left";
      }
  }
  return "";
}
