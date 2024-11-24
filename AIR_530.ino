#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Define the RX and TX pins for SoftwareSerial
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

// Create a TinyGPSPlus object
TinyGPSPlus gps;

// Set up SoftwareSerial to communicate with the GPS
SoftwareSerial ss(RXPin, TXPin);

void setup() 
{
  // Start serial communication at 115200 baud for the serial monitor
  Serial.begin(9600);
  
  // Start the SoftwareSerial communication at GPS baud rate (9600)
  ss.begin(GPSBaud);

  // Print setup message
  Serial.println(F("GPS Coordinates and Time Retrieval"));
  Serial.println(F("Using TinyGPSPlus Library"));
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void loop() 
{
  // Continuously read data from the GPS and process it
  while (ss.available() > 0) {
    gps.encode(ss.read());  // Feed the data into the TinyGPSPlus object
  }

  // If we have updated coordinates and time, print them
  //if (gps.location.isUpdated()) {
    // Print Latitude and Longitude
    Serial.print(F("Latitude: "));
    Serial.print(gps.location.lat(), 6);  // Print latitude with 6 decimal places
    Serial.print(F(" Longitude: "));
    Serial.println(gps.location.lng(), 6);  // Print longitude with 6 decimal places
  //}

 // if (gps.time.isUpdated()) {
    // Print UTC Time (hours, minutes, seconds)
    Serial.print(F("Time (UTC): "));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    Serial.println(gps.time.centisecond());  // Centisecond is a fraction of a second
  //}
  // Add a small delay to make output readable
  //delay(500);
}
