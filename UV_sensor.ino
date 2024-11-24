// Define the pin for the UV sensor
const int uvSensorPin = A1; // Connect the UV sensor's analog output to pin A0

// Voltage reference of Arduino
const float referenceVoltage = 5.0; // Typically 5V for most Arduino boards

// Variable to store the last time the sensor was read
unsigned long previousMillis = 0;
// Interval for sensor reading in milliseconds
const unsigned long interval = 500; // 500ms

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Call the function to handle UV sensor reading
  handleUVSensorReading();

  // Other tasks can be performed here without being blocked
}

// Function to read the UV sensor and print the result
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
    float voltage = sensorValue * (referenceVoltage / 1023.0);

    // Optional: Estimate UV intensity (depends on sensor specifications)
    // Example: GUVA-S12SD has a conversion factor to UV index, e.g., 0.1V = 1 UV Index
    // float uvIndex = voltage / 0.1; // Replace with your sensor's data sheet value

    // Print the voltage and UV intensity
    Serial.print("Sensor Value: ");
    Serial.print(sensorValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage);
    Serial.println(" ");
    // Serial.print(" V | UV Index: ");
    // Serial.println(uvIndex); // Uncomment if calculating UV index
  }
}
