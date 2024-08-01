const int tempSensorPin = A0; // Pin where the temperature sensor is connected
const int ledPin = 13;        // Built-in LED pin
const int buttonPin = 2;      // Pin where the button is connected

volatile bool readTemperatureFlag = false;

void setup() {
  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Initialize the button pin as an input with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // Attach an interrupt to the button pin
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
    // Otherwise, turn the LED off
    digitalWrite(ledPin, LOW);
  }
}

void loop() {
  if (readTemperatureFlag) {
    readTemperatureFlag = false;
    checkTemperature();
  }
}
