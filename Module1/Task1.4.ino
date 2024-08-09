const int tempSensorPin = A0;  // Temperature sensor pin
const int forceSensorPin = A2; // Force sensor pin (analog)
const int buttonPin = 2;       // Button pin
const int ledPin = 13;         // Built-in LED pin (for temperature)
const int forceLedPin = 12;    // New LED pin (for force sensor)
const int pirSensorPin = 3;    // PIR Sensor pin
const int pirLedPin = 11;      // LED for PIR sensor

volatile bool readTemperatureFlag = false;
volatile bool forceDetectedFlag = false;
volatile bool pirDetectedFlag = false;

const int forceThreshold = 105; // Threshold value for force sensor (corresponding to 5N)
const unsigned long debounceDelay = 1000; // Debounce delay in milliseconds

unsigned long lastForceReadingTime = 0; // Last time force was read

void setup() {
    // Initialize the LED pins as outputs
    pinMode(ledPin, OUTPUT);         // Built-in LED for temperature
    pinMode(forceLedPin, OUTPUT);    // Additional LED for force sensor
    pinMode(pirLedPin, OUTPUT);      // LED for PIR sensor

    // Initialize the button pin as an input with pull-up resistor
    pinMode(buttonPin, INPUT_PULLUP);

    // Initialize the force sensor pin as an input
    pinMode(forceSensorPin, INPUT);

    // Initialize the PIR sensor pin as an input
    pinMode(pirSensorPin, INPUT);

    // Initialize serial communication at 9600 bits per second
    Serial.begin(9600);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(buttonPin), triggerReadTemperature, FALLING);
    attachInterrupt(digitalPinToInterrupt(pirSensorPin), triggerPIRDetection, CHANGE);

    // Set up Timer1 interrupt
    noInterrupts(); // Disable all interrupts
    TCCR1A = 0; // Set Timer1 to normal mode
    TCCR1B = 0;
    TCNT1 = 0; // Initialize counter value to 0
    OCR1A = 50000; // Set compare match register for 3Hz increments
    TCCR1B |= (1 << WGM12); // Turn on CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10); // Set CS12 and CS10 bits for 1024 prescaler
    TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
    interrupts(); // Enable all interrupts
}

void triggerReadTemperature() {
    readTemperatureFlag = true;
}

void triggerPIRDetection() {
    pirDetectedFlag = true;
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
    }
    else {
        digitalWrite(ledPin, LOW);
    }
}

void checkForce() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastForceReadingTime >= debounceDelay) {
        int forceValue = analogRead(forceSensorPin);
        Serial.print("Force Value: ");
        Serial.println(forceValue);

        if (forceValue > forceThreshold) {
            forceDetectedFlag = true;
            digitalWrite(forceLedPin, HIGH);
        }
        else {
            forceDetectedFlag = false;
            digitalWrite(forceLedPin, LOW);
        }

        if (forceDetectedFlag) {
            Serial.println("Force detected!");
        }

        lastForceReadingTime = currentMillis;
    }
}


void checkPIR() {
    if (pirDetectedFlag) {
        pirDetectedFlag = false;
        digitalWrite(pirLedPin, !digitalRead(pirLedPin)); // Toggle PIR LED
        Serial.println("Motion detected!");
    }
}

ISR(TIMER1_COMPA_vect) {
    // Timer1 interrupt service routine
    // Toggle the LED connected to pin 13
    digitalWrite(ledPin, !digitalRead(ledPin));
}

void loop() {
    if (readTemperatureFlag) {
        readTemperatureFlag = false;
        checkTemperature();
    }

    checkForce();
    checkPIR();
}
