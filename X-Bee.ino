// Pin definitions
const int xbeeRxPin = 19; // RX pin for XBee (connected to XBee TX)
const int xbeeTxPin = 18; // TX pin for XBee (connected to XBee RX)
const int ledPin = 13;    // LED connected to pin 13

void setup() {
  // Initialize Serial1 for XBee communication
  Serial1.begin(9600); // Make sure the baud rate matches your XBee module
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Check if data is available from the XBee
  if (Serial1.available() > 0) {
    char receivedChar = Serial1.read(); // Read the incoming character

    // If the character 'A' is received, turn on the LED
    if (receivedChar == 'A') {
      digitalWrite(ledPin, HIGH); // Turn LED on
    } else {
      digitalWrite(ledPin, LOW);  // Turn LED off
    }
  }
}
