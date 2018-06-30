
     
#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

float precision = 0.02; 

float currentLatitude; 
float currentLongitude; 
float savedLat;
float savedLong;
float savedLatitude;
float savedLongitude;
const int curvesNumber = 4;
// Curves data: Longtitude, Latitude and Angle
float curveLat[curvesNumber] = {0.0,0.0,0.0,0.0};
float curveLong[curvesNumber] = {0.0,0.0,0.0,0.0};
int curveAngle[curvesNumber] ={0,0,0,0};



// set pin numbers:
const int buttonPin = PUSH1;         // the number of the pushbutton pin
const int ledPin =  YELLOW_LED;      // the number of the LED pin
int buttonState = 0;                 // variable for reading the pushbutton status
float speed ;                          // variable for reading the speed
float distance (float Lat1,float Long1,float Lat2,float Long2); //measures the distance between two points
int nearCurve();                     // detects wether the driver has come close to a curve or not and returns the number of that curve

void setup()
{
  savedLat = -1.0;
  savedLong = -1.0;

  //setup button
  //initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  pinMode(GREEN_LED,OUTPUT);
  pinMode(2,INPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(40,OUTPUT);
  digitalWrite(8,LOW);
  digitalWrite(7, HIGH);
  digitalWrite(40,LOW);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);     
  
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
     
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
 // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
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
}

void loop() // run over and over again
{
  //compare current position with saved positions:
  if (nearCurve()>=0)
  {
   // turn LED on:    
    digitalWrite(ledPin, HIGH);
    speed= analogRead(A10);
    analogWrite(37,speed-512);
    analogWrite(38,512-speed);
  } 
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
    analogWrite(37,0);
    analogWrite(38,0);
  }
    
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
 // if (GPSECHO)
   // if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    GPS.lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
    //update position in variables
    currentLatitude = GPS.latitude; //six decimals equal a precision of about 0.11m!
    currentLongitude = GPS.longitude;
  
 // if we have a fix light up the green LED on the board
  if(GPS.fix){
    digitalWrite(GREEN_LED,HIGH);
    }else{
      digitalWrite(GREEN_LED,LOW);
      }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    Serial.print("Saved lat: ");
    Serial.print(savedLat,4);
    Serial.print("\n");
    Serial.print("Saved long: ") ;
    Serial.print(savedLong,4);
    Serial.print("\n");
    Serial.print("Precision: ");
    Serial.print(precision,4);
    Serial.print("\n");
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      //Serial.print("Location (in degrees, works with Google Maps): ");
     // Serial.print(GPS.latitudeDegrees, 4);
      //Serial.print(", "); 
      //Serial.println(GPS.longitudeDegrees, 4);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
     // Serial.print("Altitude: "); Serial.println(GPS.altitude);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}
float distance (float Lat1,float Long1,float Lat2,float Long2){
  return sqrt(((Lat1-Lat2)*(Lat1-Lat2))+((Long1-Long2)*(Long1-Long2)));
  }
 int nearCurve(){
 for(int i=0; i<curvesNumber; i++){
   if(distance(curveLat[i],currentLatitude,curveLong[i],currentLongitude)>precision){
    return i;
    }
  }
  return -1; //return -1 if far from any curve
 }
