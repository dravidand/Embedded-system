#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
double latitude = 0.0;
double longitude = 0.0;

const int trigPin = 9;
const int echoPin = 8;
const int ledPin = 13;  // Pin for the LED

float duration, distance;
unsigned long previousMillis = 0;  
unsigned long interval = 100;  

const int A0Pin = A0;
const int ledPins[] = {40, 41, 42, 43};

char command = ' ';
long lastCommandTime = 0; // Last command timestamp for debounce
const long debounceTime = 100; // Debounce duration (milliseconds)
const int xbeeRxPin = 19;  // RX pin for XBee (connected to XBee TX)
const int xbeeTxPin = 18;  // TX pin for XBee (connected to XBee RX)

const int motor1In1Pin = 22;  // IN1 for Motor 1 (Direction control)
const int motor1In2Pin = 23;  // IN2 for Motor 1 (Direction control)
const int motor1EnPin = 6;    // ENA for Motor 1 (PWM control for speed)

const int motor2In3Pin = 24;  // IN3 for Motor 2 (Direction control)
const int motor2In4Pin = 25;  // IN4 for Motor 2 (Direction control)
const int motor2EnPin = 7;    // ENB for Motor 2 (PWM control for speed)

const int motor3In1Pin = 33;  // IN1 for Motor 1 (Direction control)
const int motor3In2Pin = 32;  // IN2 for Motor 1 (Direction control)
const int motor3EnPin = 2;    // ENA for Motor 1 (PWM control for speed)

const int motor4In3Pin = 34;  // IN3 for Motor 2 (Direction control)
const int motor4In4Pin = 35;  // IN4 for Motor 2 (Direction control)
const int motor4EnPin = 3;    // ENB for Motor 2 (PWM control for speed)

int motorSpeed = 9;

int pinStatus;

const int chipSelect = 53;
File dataFile;

const int uvSensorPin = A4;
const float referenceVoltage = 5.0;
float uvIndex = 0;
float uvValue = 0;      

//Optimization
unsigned long lastXBeeRead = 0;
unsigned long lastGPSRead = 0;
unsigned long lastSDWrite = 0;
unsigned long lastSensorCheck = 0;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.println("Ultrasonic control initialized.");  
  for (int i = 0; i < 4; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
  Serial.begin(9600);
  Serial1.begin(9600);
  ss.begin(GPSBaud);
  Serial.println("Serial control initialized.");

  pinMode(motor1In1Pin, OUTPUT);
  pinMode(motor1In2Pin, OUTPUT);
  pinMode(motor1EnPin, OUTPUT);  // ENA pin for Motor 1 speed control

  pinMode(motor2In3Pin, OUTPUT);
  pinMode(motor2In4Pin, OUTPUT);
  pinMode(motor2EnPin, OUTPUT);  // ENB pin for Motor 2 speed control

  pinMode(motor3In1Pin, OUTPUT);
  pinMode(motor3In2Pin, OUTPUT);
  pinMode(motor3EnPin, OUTPUT);  // ENA pin for Motor 3 speed control

  pinMode(motor4In3Pin, OUTPUT);
  pinMode(motor4In4Pin, OUTPUT);
  pinMode(motor4EnPin, OUTPUT);  // ENB pin for Motor 4 speed control

  // Initialize motors as off
  digitalWrite(motor1In1Pin, LOW);
  digitalWrite(motor1In2Pin, LOW);
  analogWrite(motor1EnPin, 0);  // Initially set motor 1 to stop (PWM = 0)

  digitalWrite(motor2In3Pin, LOW);
  digitalWrite(motor2In4Pin, LOW);
  analogWrite(motor2EnPin, 0);  // Initially set motor 2 to stop (PWM = 0)

  // Initialize motors as off
  digitalWrite(motor3In1Pin, LOW);
  digitalWrite(motor3In2Pin, LOW);
  analogWrite(motor3EnPin, 0);  // Initially set motor 3 to stop (PWM = 0)

  digitalWrite(motor4In3Pin, LOW);
  digitalWrite(motor4In4Pin, LOW);
  analogWrite(motor4EnPin, 0);  // Initially set motor 4 to stop (PWM = 0)

  Serial.println("Motor control initialized.");
  
}

void loop() {

    // Time-sensitive: Check for XBee data frequently
    if (millis() - lastXBeeRead > 10) { // Every 10 ms
        readXBeeData();
        lastXBeeRead = millis();
    }

    // Moderate priority: GPS data processing
    if (millis() - lastGPSRead > 1000) { // Every 1 second
        readAndProcessGPSData();
        lastGPSRead = millis();
    }

    // SD card writing (less frequent)
    if (millis() - lastSDWrite > 2000) { // Every 2 seconds
        sdCardWritingData();
        lastSDWrite = millis();
    }

    // Sensor reading and LED control
    if (millis() - lastSensorCheck > 100) { // Every 100 ms
        getDistance();
        checkA0AndControlLEDs();
        handleUVSensorReading();
        lastSensorCheck = millis();
        if (distance < 10) {
          blinkHazardLight();
          setMotorSpeed('9');
        } 
        else {
          setMotorSpeed('9');
          digitalWrite(ledPin, LOW);  // Turn off the LED when distance is not less than 10 cm
        }
        Serial1.print("Motor speed set to: ");
        Serial1.println(motorSpeed);
    }
  

}

void getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;  // Calculate the distance in cm
  Serial.print("Distance: ");
  Serial.println(distance);
  Serial1.print("Distance: ");
  Serial1.println(distance);
}

void blinkHazardLight() {
  unsigned long currentMillis = millis();  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  
    int ledState = digitalRead(ledPin);
    digitalWrite(ledPin, !ledState);
  }
}

void checkA0AndControlLEDs() {
  int A0Value = digitalRead(A0Pin);
  //Serial.println(A0Value);
  if (A0Value == HIGH) {  
    // Turn on LEDs 50, 51, 52, 53
    for (int i = 0; i < 4; i++) {
      digitalWrite(ledPins[i], HIGH);
    }
  } else {
    for (int i = 0; i < 4; i++) {
      digitalWrite(ledPins[i], LOW);
    }
  }
}

void readAndProcessGPSData() {
    // Non-blocking: Read only one character per call to avoid long blocking periods
    while (ss.available() > 0) {
        gps.encode(ss.read()); // Encode GPS data
    }

    // Process only if GPS data is updated
    if (gps.location.isUpdated() || gps.time.isUpdated() || gps.date.isUpdated()) {
        // Speed
        logGPSData("Speed (km/h): ", gps.speed.kmph());

        // Location
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        logGPSData("Latitude: ", latitude, 6);
        logGPSData("Longitude: ", longitude, 6);

        // Time
        logGPSData("Time (UTC): ", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());

        // Date
        logGPSData("Date (UTC): ", gps.date.day(), gps.date.month(), gps.date.year());
    }
}

// Helper function for general GPS data logging
void logGPSData(const char* label, double value, int decimalPlaces) {
    Serial.print(label);
    Serial.println(value, decimalPlaces);
    Serial1.print(label);
    Serial1.println(value, decimalPlaces);
}

void logGPSData(const char* label, double value) {
    Serial.print(label);
    Serial.println(value, 2);  // Default to 2 decimal places
    Serial1.print(label);
    Serial1.println(value, 2);
}

void logGPSData(const char* label, int hour, int minute, int second, int centisecond) {
    Serial.print(label);
    Serial1.print(label);
    
    Serial.print(hour);
    Serial1.print(hour);
    Serial.print(":");
    Serial1.print(":");
    Serial.print(minute);
    Serial1.print(minute);
    Serial.print(":");
    Serial1.print(":");
    Serial.print(second);
    Serial1.print(second);
    Serial.print(".");
    Serial1.print(".");
    Serial.println(centisecond);
    Serial1.println(centisecond);
}

void logGPSData(const char* label, int day, int month, int year) {
    Serial.print(label);
    Serial1.print(label);
    
    Serial.print(day);
    Serial1.print(day);
    Serial.print("/");
    Serial1.print("/");
    Serial.print(month);
    Serial1.print(month);
    Serial.print("/");
    Serial1.print("/");
    Serial.println(year);
    Serial1.println(year);
}

  

void readXBeeData() {
  if (Serial1.available()) {
    char newCommand = Serial1.read();
    if (newCommand != command) {
      command = newCommand;
      //lastCommandTime = millis(); // Reset debounce timer
    }
        while (Serial1.available()) { // Flush buffer to prevent processing old data
        Serial1.read();
    }
  }

  // Stop motors if no command is received recently
  //if (millis() - lastCommandTime > debounceTime) {
    if (command == ' ') {
      //command = ' '; // Reset command after debounce time
      // Optionally, you can call `doNothing()` here or stop motors
      doNothing();  // Stops motors if no command is received
    }
  //}
  Serial.print("----------------------------->");
  Serial.println(command);

  Serial1.print("----------------------------->");
  Serial1.println(command);
  
  switch (command){
      case 'W':
        setMotorSpeed(command);
        moveForward();
        break;
        
      case 'A':
        setMotorSpeed(command);
        turnLeft();
        break;
        
      case 'S':
        setMotorSpeed(command);
        moveBackward();
        break;
        
      case 'D':
        setMotorSpeed(command);
        turnRight();
        break;
        
      default:
        setMotorSpeed(command);
        doNothing();
        break;
      }
    /*if (incomingByte == 'W' || incomingByte == 'w') {
      moveForward();
      //lastDirection = 'W';
    } 
    else if (incomingByte == 'A' || incomingByte == 'a') {
      turnLeft();
      //lastDirection = 'A';
    } 
    else if (incomingByte == 'S' || incomingByte == 's') {
      moveBackward();
      //lastDirection = 'S';
    } 
    else if (incomingByte == 'D' || incomingByte == 'd') {
      turnRight();
      //lastDirection = 'D';
    }
    else if (incomingByte == 'P' || incomingByte == 'p') {
      doNothing();
      //lastDirection = 'D';
    }
    // If a number is received (0-9), adjust motor speed
    else if (incomingByte >= '0' && incomingByte <= '9') {
      setMotorSpeed(incomingByte);
    }
    if(pinStatus == HIGH){
      doNothing();
    }
  }*/
}
void setMotorSpeed(char incomingByte) {
    if (incomingByte >= '0' && incomingByte <= '9') {
    // Convert the numeric character to an integer and map it to PWM range
    motorSpeed = map(incomingByte - '0', 0, 9, 0, 255);  // Map 0-9 to 0-255 for PWM
    Serial.print("Motor speed set to: ");
    Serial.println(motorSpeed);
  }
}
void turnRight() {
  // Motor 1 Direction (Backward)
  digitalWrite(motor1In1Pin, HIGH);
  digitalWrite(motor1In2Pin, LOW);
  digitalWrite(motor3In1Pin, HIGH);
  digitalWrite(motor3In2Pin, LOW);
  analogWrite(motor1EnPin, motorSpeed);  // Set motor 1 speed using PWM
  analogWrite(motor3EnPin, motorSpeed);  // Set motor 3 speed using PWM
  
  // Motor 2 Direction (Forward)
  digitalWrite(motor2In3Pin, HIGH);
  digitalWrite(motor2In4Pin, LOW);
  digitalWrite(motor4In3Pin, HIGH);
  digitalWrite(motor4In4Pin, LOW);
  analogWrite(motor2EnPin, motorSpeed);  // Set motor 2 speed using PWM
  analogWrite(motor4EnPin, motorSpeed);  // Set motor 4 speed using PWM

  Serial.println("Turning right.");
  Serial1.println("Turning right.");
}

void moveBackward() {
  // Motor 1 Direction (Backward)
 
  digitalWrite(motor1In1Pin, HIGH);
  digitalWrite(motor1In2Pin, LOW);
  digitalWrite(motor3In1Pin, LOW);
  digitalWrite(motor3In2Pin, HIGH);
  analogWrite(motor1EnPin, motorSpeed);  // Set motor 1 speed using PWM
  analogWrite(motor3EnPin, motorSpeed);  // Set motor 3 speed using PWM
  
  // Motor 2 Direction (Forward)
  digitalWrite(motor2In3Pin, HIGH);
  digitalWrite(motor2In4Pin, LOW);
  digitalWrite(motor4In3Pin, LOW);
  digitalWrite(motor4In4Pin, HIGH);
  analogWrite(motor2EnPin, motorSpeed);  // Set motor 2 speed using PWM
  analogWrite(motor4EnPin, motorSpeed);  // Set motor 4 speed using PWM

  Serial.println("Moving backward.");
  Serial1.println("Moving backward.");
}

void turnLeft() {
  // Motor 1 Direction (Forward)
  digitalWrite(motor1In1Pin, LOW);
  digitalWrite(motor1In2Pin, HIGH);
  digitalWrite(motor3In1Pin, LOW);
  digitalWrite(motor3In2Pin, HIGH);
  analogWrite(motor1EnPin, motorSpeed);  // Set motor 1 speed using PWM
  analogWrite(motor3EnPin, motorSpeed);  // Set motor 3 speed using PWM
  
  // Motor 2 Direction (Backward)
  digitalWrite(motor2In3Pin, LOW);
  digitalWrite(motor2In4Pin, HIGH);
  digitalWrite(motor4In3Pin, LOW);
  digitalWrite(motor4In4Pin, HIGH);
  analogWrite(motor2EnPin, motorSpeed);  // Set motor 2 speed using PWM
  analogWrite(motor4EnPin, motorSpeed);  // Set motor 4 speed using PWM

  Serial.println("Turning left.");
  Serial1.println("Turning left.");
}

void moveForward() {
    digitalWrite(motor1In1Pin, LOW);
  digitalWrite(motor1In2Pin, HIGH);
  digitalWrite(motor3In1Pin, HIGH);
  digitalWrite(motor3In2Pin, LOW);
  analogWrite(motor1EnPin, motorSpeed);  // Set motor 1 speed using PWM
  analogWrite(motor3EnPin, motorSpeed);  // Set motor 3 speed using PWM
  
  // Motor 2 Direction (Backward)
  digitalWrite(motor2In3Pin, LOW);
  digitalWrite(motor2In4Pin, HIGH);
  digitalWrite(motor4In3Pin, HIGH);
  digitalWrite(motor4In4Pin, LOW);
  analogWrite(motor2EnPin, motorSpeed);  // Set motor 2 speed using PWM
  analogWrite(motor4EnPin, motorSpeed);  // Set motor 4 speed using PWM
  
  Serial.println("Moving forward.");
  Serial1.println("Moving forward.");
}
void doNothing() {
  // Motor 1 Direction (Forward)
  digitalWrite(motor1In1Pin, LOW);
  digitalWrite(motor1In2Pin, LOW);
  digitalWrite(motor3In1Pin, LOW);
  digitalWrite(motor3In2Pin, LOW);
  analogWrite(motor1EnPin, 0);  // Set motor 1 speed using PWM
  analogWrite(motor3EnPin, 0);  // Set motor 3 speed using PWM
  
  // Motor 2 Direction (Backward)
  digitalWrite(motor2In3Pin, LOW);
  digitalWrite(motor2In4Pin, LOW);
  digitalWrite(motor4In3Pin, LOW);
  digitalWrite(motor4In4Pin, LOW);
  analogWrite(motor2EnPin, 0);  // Set motor 2 speed using PWM
  analogWrite(motor4EnPin, 0);  // Set motor 4 speed using PWM

  Serial.println("No movement");
  Serial1.println("No movement");
}

void stopMotors() {
  // Stop both motors
  digitalWrite(motor1In1Pin, LOW);
  digitalWrite(motor1In2Pin, LOW);
  analogWrite(motor1EnPin, 0);  // Set motor 1 to stop (PWM = 0)
  
  digitalWrite(motor2In3Pin, LOW);
  digitalWrite(motor2In4Pin, LOW);
  analogWrite(motor2EnPin, 0);  // Set motor 2 to stop (PWM = 0)

  Serial.println("Motors stopped.");
  Serial1.println("Motors stopped.");
}

void sdCardWritingData() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (true); // Stop here if initialization fails
  }
  Serial.println("SD card initialized.");
  // Open the file for appending
  dataFile = SD.open("log.txt", FILE_WRITE);

  if (dataFile) {
    // Use a String buffer to optimize writing
    String logEntry = "";

    logEntry += F("Date (UTC): ");
    logEntry += gps.date.day();
    logEntry += F("/");
    logEntry += gps.date.month();
    logEntry += F("/");
    logEntry += gps.date.year();
    logEntry += F("<--->");

    logEntry += F("Timestamp RTC: ");
    logEntry += gps.time.hour();
    logEntry += F(":");
    logEntry += gps.time.minute();
    logEntry += F(":");
    logEntry += gps.time.second();
    logEntry += F(":");
    logEntry += gps.time.centisecond();
    logEntry += F("<--->");

    logEntry += F("Latitude: ");
    logEntry += String(latitude, 6); // Convert to String with 6 decimals
    logEntry += F("<--->");
    logEntry += F("Longitude: ");
    logEntry += String(longitude, 6);
    logEntry += F("<--->");

    logEntry += F("UV Radiation: ");
    logEntry += uvIndex;
    logEntry += F("<--->");

    logEntry += F("Distance: ");
    logEntry += distance;
    logEntry += F("<--->");
    
    logEntry += F("Actual Speed Km/h: ");
    logEntry += gps.speed.kmph();
    logEntry += F("<--->");

    logEntry += F("Motor Speed: ");
    logEntry += motorSpeed;
    logEntry += F("<--->");

    logEntry += F("X-BEE Command: ");
    logEntry += command;
    logEntry += F("\n");

    // Write logEntry to SD card
    dataFile.println(logEntry);
    dataFile.close();

    Serial1.println("Data Logged");
  } else {
    // Handle file open failure
    Serial.println("Error opening log.txt");
    Serial1.println("Check SD card");
  }
}
void handleUVSensorReading() {
  // Get the current time
  unsigned long currentMillis = millis();

  // Check if the interval has passed
  if (currentMillis - previousMillis >= interval) {
    // Save the current time as the last time the sensor was read
    previousMillis = currentMillis;

    // Read the analog value from the UV sensor
    float sensorValue = analogRead(uvSensorPin);

    // Convert the analog value to voltage
     float uvIndex = map(uvValue, 0, 1023, 0, 15); 

    // Optional: Estimate UV intensity (depends on sensor specifications)
    // Example: GUVA-S12SD has a conversion factor to UV index, e.g., 0.1V = 1 UV Index

    // Print the voltage and UV intensity
    Serial.print("Sensor Value: ");
    Serial.print(sensorValue);

    Serial.println(" ");
    Serial1.print("UV Radiation: ");
    Serial1.println(sensorValue);

    Serial1.print(" V | UV Index: ");
    Serial1.println(sensorValue); // Uncomment if calculating UV index
  }
}
