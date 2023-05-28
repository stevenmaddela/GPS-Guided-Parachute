// run the HMC5883L compass calibration test and enter results into code below under void Setup() compass.setOffset(0,0);      

#include <DFRobot_QMC5883.h>
#include "Wire.h"                                                
#include <TinyGPS++.h>                                            // http://arduiniana.org/libraries/tinygpsplus/
                                                                  // TinyGPS++ uses serial 2
                                                                  // 9600-baud gps on pins 16(tx) and 17(rx).                                                                  

//******************************************************************************************************                                                                  

// GPS
int GPS_Course;                                                    // gps's determined course to destination
int Number_of_SATS;                                                // number of satellites acquired
TinyGPSPlus gps;                                                   
                                                                   // pin 17 rx on arduino is connected to the TX on the GPS
                                                                   // pin 16 tx on arduino is connected to the RX on the GPS
                                                                   // vcc on gps to 5v on arduino
                                                                   // gnd on gps to gnd on arduino                                    
//******************************************************************************************************
//Motors

int turn_Speed = 175;                                              // motor speed compass to turn left and right
int mtr_Spd = 250;                                                 // motor speed moving forward and reverse
int enA = 10;
int in1 = 9;
int in2 = 8;
// motor two
int enB = 5;
int in3 = 7;
int in4 = 6;
//******************************************************************************************************
// Compass
DFRobot_QMC5883 compass;
int16_t mx, my, mz;                                                // x,y,z axis from compass 
int desired_heading;                                               // value for desired heading
int compass_heading;                                               // value calculated from compass readings
int compass_dev = 5;                                               // the amount of deviation that is allowed in the compass heading - Adjust as Needed
                                                                   // setting this variable too low will cause the robot to continuously pivot left and right
                                                                   // setting this variable too high will cause the robot to veer off course
                                                                   //vcc to plus on motor board
                                                                   //gnd to minus on motor board
                                                                   //scl 21 arduino to scl compass
                                                                   //sda 20 arduino to sda compass

//******************************************************************************************************
//Possible Radio variables

//*****************************************************************************************************

// GPS Locations

unsigned long Distance_To_Home;                                    // distance to destination
double MaxHeight =-3;
bool MaxHeightAchieved=false;
double CurrentHeight=0;
int ac =0;                                                         // GPS array counter
double Home_LATarray[50];                                          // destination Latitude - 5 waypoints
double Home_LONarray[50];                                          // destination Longitude - 5 waypoints

int increment = 0;

void setup() 
{  
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer
  Serial1.begin(9600);                                             // Serial 1 is for GPS communication 
  
  // Compass
  //  Wire.begin();                                                    // Join I2C bus used for the HMC5883L compass
   compass.begin();                                                 // initialize the compass (HMC5883L)
   compass.setRange(QMC5883_RANGE_2GA);
   compass.setMeasurementMode(QMC5883_CONTINOUS); 
   compass.setDataRate(QMC5883_DATARATE_50HZ);
   compass.setSamples(QMC5883_SAMPLES_8);
   //compass.setOffset(0,0);                                          // Set calibration offset 

   Startup();                                                       // Run the Startup procedure on power-up one time
}

//********************************************************************************************************
// Main Loop

void loop()
{ 
  getGPS();                                                        // Update the GPS location
  //gpsInfo();
  getCompass();                                                    // Update the Compass Heading
  CurrentHeight = gps.altitude.meters();
  Serial.print("Max Height: ");
    Serial.print(MaxHeight);
  Serial.print(" -- Current Height: ");
    Serial.println(CurrentHeight);
  if ((MaxHeight <= CurrentHeight+3 && CurrentHeight > MaxHeight) && MaxHeightAchieved ==false)
  {
    MaxHeight = CurrentHeight;
  }
  else if(((MaxHeight > CurrentHeight+5) && CurrentHeight > 100) && MaxHeightAchieved ==false){//1250 feet up where 1000 is regular elevation??
    MaxHeightAchieved = true;      
    goWaypoint();
    exit(0);
  }
  else{

  }
}

void Startup()
{           
  for (int i=5; i >= 1; i--)                       // Count down for X seconds
      {         
        Serial.print("Wait for Startup... "); 
        Serial.println(i);
        delay(1000);                                   // Delay for X seconds
      }    
  Serial.println("\nSearching for Satellites "); 
      int timer = 0;
  while (Number_of_SATS <= 3)                         // Wait until x number of satellites are acquired before starting main loop
  {                                  
    getGPS();  

                                           // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired       

  }  
  CurrentHeight = gps.altitude.meters();  
  setWaypoint();                                      // set intial waypoint to current location
  ac = 0;                                             // zero array counter
  
  gpsInfo();
}    

void getGPS()                                                 // Get Latest GPS coordinates
{
    while (Serial1.available() > 0){
      //Serial.write(Serial1.read());
      gps.encode(Serial1.read());
    }
} 

void getCompass()                                               // get latest compass value
 {  

  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
 
  if(heading < 0)
     heading += 2 * M_PI;      
  compass_heading = (int)(heading * 180/M_PI);                   // assign compass calculation to variable (compass_heading) and convert to integer to remove decimal places                                                              
 }

void setWaypoint()                                            // Set up to 5 GPS waypoints
{
    Serial.println("GPS Waypoint Set");
    getGPS();                                                 // get the latest GPS coordinates
    getCompass();                                             // update latest compass heading     
                                               
    Home_LATarray[ac] = gps.location.lat();                   // store waypoint in an array   
    Home_LONarray[ac] = gps.location.lng();                   // store waypoint in an array   
                                                              
    Serial.print("Waypoint: ");
    Serial.print(Home_LATarray[0],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0],6);
    CurrentHeight = gps.altitude.meters();
    Serial.print("Current Height: ");
    Serial.println(CurrentHeight);
    ac++;                                                       // increment array counter        
}

void clearWaypoints()
{
   memset(Home_LATarray, 0, sizeof(Home_LATarray));             // clear the array
   memset(Home_LONarray, 0, sizeof(Home_LONarray));             // clear the array
   ac = 0;   
   Serial1.print("GPS Waypoint Cleared");                      // display waypoints cleared
  
}


void gpsInfo()                                                  // displays Satellite data to user
{
   Number_of_SATS = (int)(gps.satellites.value());         //Query Tiny GPS for the number of Satellites Acquired 
   Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination    
   Serial.print("Lat:");
   Serial.print(gps.location.lat(),6);
   Serial.print(" Lon:");
   Serial.print(gps.location.lng(),6);
   Serial.print(" ");
   Serial.print(Number_of_SATS); 
   Serial.print(" SATs Acquired");
   Serial.print(Distance_To_Home);
   Serial.print("m"); 
   Serial.print("Distance to Home ");
   Serial.println(Distance_To_Home);
}
 

void goWaypoint()
{   
  Serial.println("Go to Waypoint");
  while (true)  
  {                                                                // Start of Go_Home procedure 
   getCompass();                                                    // Update Compass heading                                          
   getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location// delay time changed from 100 to 10
   if (millis() > 5000 && gps.charsProcessed() < 10){                // If no Data from GPS within 5 seconds then send error
    Serial.println(F("No GPS data: check wiring"));
   }     
 
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[0], Home_LONarray[0]);  //Query Tiny GPS for Distance to Destination
  GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),Home_LATarray[0],Home_LONarray[0]);                               //Query Tiny GPS for Course to Destination   
  Serial.print("Distance To Home: ");
  Serial.print(Distance_To_Home);
  Serial.print(" GPS_Course: ");
  Serial.print(GPS_Course);
  Serial.print(" Compass Heading: ");
  Serial.print(compass_heading);
   Serial.print(" Waypoint: ");
    Serial.print(Home_LATarray[0],6);
    Serial.print(" ");
    Serial.print(Home_LONarray[0],6);
    Serial.print(" Current Lat:");
   Serial.print(gps.location.lat(),6);
   Serial.print(" Lon:");
   Serial.println(gps.location.lng(),6);

    if (Distance_To_Home == 0 || gps.altitude.meters()<2)                                   // If the Vehicle has reached it's Destination, then Stop
        {
         StopGliding();                                               // Stop the robot after each waypoint is reached
        clearWaypoints();
        Serial.println("You have arrived!");                    // Print to Bluetooth device - "You have arrived"          
        break;                                                   // Break from Go_Home procedure and send control back to the Void Loop         
        }   
    if ( abs(GPS_Course - compass_heading) <= 15)                  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward                                                                  
                                                                  // otherwise find the shortest turn radius and turn left or right  
      {
         Forward();                                               // Go Forward
      } 
      else 
      {                                                       
         int x = (GPS_Course - 360);                           // x = the GPS desired heading - 360
         int y = (compass_heading - (x));                      // y = the Compass heading - x
         int z = (y - 360);                                    // z = y - 360
         
          if ((z <= 180) && (z >= 0))                           // if z is less than 180 and not a negative value then turn left otherwise turn right
          { SlowLeftTurn();  }
          else
          { SlowRightTurn(); }               
       } 
    

   }                                                              // End of While Loop

}

void StopGliding()
{
  // motor1.run(RELEASE);                                                         
  // motor2.run(RELEASE);   
}

void Forward()
{
  // motor1.setSpeed(mtr_Spd);                                                   
  // motor2.setSpeed(mtr_Spd);                            
  // motor1.run(FORWARD);                                                         // go forward all wheels 
  // motor2.run(FORWARD);
}

void SlowLeftTurn()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
  // accelerate from zero to maximum speed

 for (int i = 0; i < 30; i++) { 
analogWrite(enA, i); 
analogWrite(enB, i); 
delay(20); 
}  
digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
}

void SlowRightTurn()
{
   
  // motor1.setSpeed(turn_Speed);                                                  
  // motor2.setSpeed(turn_Speed);                      
  // motor1.run(FORWARD);                                                           
  // motor2.run(BACKWARD);
   digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
  // accelerate from zero to maximum speed

 for (int i = 0; i < 30; i++) { 
analogWrite(enA, i); 
analogWrite(enB, i); 
delay(20); 
}  
digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
}
