
const int tempSensorPin = A0; // Pin where temperature sensor is connected
const int ledPin = 13;        // Built-in LED pin

void setup() {
  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
}

void loop() {
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

  // Wait for a second before taking another reading
  delay(1000);
}
