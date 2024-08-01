const int tempSensorPin = A0;  // Temperature sensor pin
const int forceSensorPin = A2; // Force sensor pin (analog)
const int buttonPin = 2;       // Button pin
const int ledPin = 13;         // Built-in LED pin (for temperature)
const int forceLedPin = 12;    // New LED pin (for force sensor)

volatile bool readTemperatureFlag = false;
volatile bool forceDetectedFlag = false;

// Threshold values
const int forceThreshold = 512; 
const unsigned long debounceDelay = 1000; // Debounce delay in milliseconds

unsigned long lastForceReadingTime = 0; // Last time force was read

void setup() {
  // Initialize the LED pins as outputs
  pinMode(ledPin, OUTPUT);         // Built-in LED for temperature
  pinMode(forceLedPin, OUTPUT);    // Additional LED for force sensor

  // Initialize the button pin as an input with pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize the force sensor pin as an input
  pinMode(forceSensorPin, INPUT);

  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(buttonPin), triggerReadTemperature, FALLING);
}

void triggerReadTemperature() {
  readTemperatureFlag = true;
}

void checkTemperature() {
  // Read the analog value from the temperature sensor
  int sensorValue = analogRead(tempSensorPin);

  // Convert the analog value to voltage
  float voltage = sensorValue * (5.0 / 1023.0);

  // Convert the voltage to temperature in Celsius
  float temperatureC = (voltage - 0.5) * 100.0;

  // Print the temperature to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  // If the temperature is below 30°C, turn the LED on
  if (temperatureC < 30.0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

void checkForce() {
  unsigned long currentMillis = millis(); // Get the current time
  if (currentMillis - lastForceReadingTime >= debounceDelay) {
    int forceValue = analogRead(forceSensorPin);

    if (forceValue < forceThreshold) {  // Check if the force value exceeds the threshold
      forceDetectedFlag = true; // Set the flag if force is detected
      digitalWrite(forceLedPin, HIGH); // Turn on the force sensor LED
    } else {
      forceDetectedFlag = false; // Reset the flag if force is not detected
      digitalWrite(forceLedPin, LOW); // Turn off the force sensor LED
    }

    if (forceDetectedFlag) {
      Serial.println("Force detected!"); // Print message to Serial Monitor
    }

    lastForceReadingTime = currentMillis; // Update the last reading time
  }
}

void loop() {
  if (readTemperatureFlag) {
    readTemperatureFlag = false;
    checkTemperature();
  }

  checkForce();
}

