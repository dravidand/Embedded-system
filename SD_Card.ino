#include <SPI.h>
#include <SD.h>

// Define pins
const int chipSelect = 53;   // Chip select for SD card
const int sensorPin = A0;    // Analog input pin

File dataFile;

// Variables for storing data
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval to log data (in milliseconds)

// Setup function
void setup() {
  Serial.begin(9600);
  
  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (true); // Stop here if initialization fails
  }
  Serial.println("SD card initialized.");

  // Create or open the data file
  dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Timestamp (ms), Voltage (V)");
    dataFile.close();
  } else {
    Serial.println("Error opening log.txt");
  }
}

// Function to read analog voltage and write to SD card
void logData() {
  int sensorValue = analogRead(sensorPin);
  float voltage = sensorValue * (5.0 / 1023.0); // Convert to voltage

  // Get timestamp using millis()
  unsigned long currentMillis = millis();

  // Open file for writing
  dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(currentMillis);
    dataFile.print(", ");
    dataFile.println(voltage);
    dataFile.close();
    Serial.print("Logged: ");
    Serial.print(currentMillis);
    Serial.print(" ms, ");
    Serial.print(voltage);
    Serial.println(" V");
  } else {
    Serial.println("Error opening log.txt for writing");
  }
}

void loop() {
  // Log data at specified intervals
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    logData();
  }
}
