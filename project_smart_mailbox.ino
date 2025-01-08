#include "VOneMqttClient.h"
#include <ESP32Servo.h>

const char* InfraredSensor = "9cfacc01-c6e6-4c89-8c82-c24c93f3550d";  // Sensor for mail delivery
const char* MailCountSensor = "b111ca4f-20f5-4be2-b05c-cd85ecd66f08";  // Sensor for mail count
const char* PushButton = "4ee47a92-9d6b-4429-adc9-91c7a7c70eb0";  // Push button
const char* ServoMotor = "d30df230-ab7b-46cf-a6a4-9e3ee5bec8f5";  // Servo motor

// Pin definitions
const int irDeliverySensorPin = 4;  // IR Sensor to detect postman
const int irInsideSensorPin = 42;   // IR Sensor to detect mail inside the box
const int greenLED = 5;             // Green LED: 
const int yellowLED = 6;            // Yellow LED: 
const int redLED = 9;               // Red LED: 
const int buttonPin = 38;           // Push button
const int servoPin = 8;             // Servo motor pin

// Components
VOneMqttClient voneClient;
Servo mailboxServo;

// Variables
bool mailPresent = false;
bool postmanDetected = false;
int mailCount = 0;
int postmanCount = 0;  // Counter for postman detection
int pushButtonPressCount = 0; // Counter for push button presses (mailbox opening)
bool mailboxOpen = false;

// Wi-Fi setup
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Callback for servo control
void triggerServo_callback(const char* actuatorDeviceId, const char* actuatorCommand) {
  Serial.println("Servo command received from cloud.");
  JSONVar commandObject = JSON.parse(actuatorCommand);
  int angle = (int)commandObject["angle"];
  mailboxServo.write(angle);
  mailboxOpen = (angle != 0);
  updateLEDs();
  voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize Wi-Fi and V-One
  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerServo_callback);

  // Set pin modes
  pinMode(irDeliverySensorPin, INPUT);
  pinMode(irInsideSensorPin, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Attach servo
  mailboxServo.attach(servoPin);
  mailboxServo.write(0);  // Ensure mailbox is closed initially

  // Initial LED state
  updateLEDs();
}

void loop() {
  // Ensure the V-One client stays connected
  if (!voneClient.connected()) {
    voneClient.reconnect();
    voneClient.publishDeviceStatusEvent(InfraredSensor, true);
    voneClient.publishDeviceStatusEvent(MailCountSensor, true);
    voneClient.publishDeviceStatusEvent(PushButton, true);
  }
  voneClient.loop();

  // Detect postman delivery
  if (digitalRead(irDeliverySensorPin) == HIGH) {
    if (!postmanDetected) {
      postmanDetected = true;
      postmanCount++;
      Serial.println("Postman detected! Checking for delivery...");
      Serial.print("Postman count: ");
      Serial.println(postmanCount);
      blinkLED(greenLED, 1); // Blink green LED once for postman detection
      voneClient.publishTelemetryData(InfraredSensor, "Obstacle", postmanCount);
    }
  } else {
    postmanDetected = false;
  }

  // Detect mail inside the mailbox
  if (digitalRead(irInsideSensorPin) == HIGH) {
    if (!mailPresent) {
      mailPresent = true;
      mailCount++;
      Serial.println("New mail detected inside the mailbox!");
      Serial.print("Mail count: ");
      Serial.println(mailCount);
      blinkLED(yellowLED, 1); // Blink yellow LED once for new mail detection
      voneClient.publishTelemetryData(MailCountSensor, "Obstacle", mailCount);
      updateLEDs();
    }
  } else {
    if (mailPresent) {
      mailPresent = false;
      updateLEDs();
    }
  }

  // Handle push button for manual control
  if (digitalRead(buttonPin) == LOW) {
    delay(200);  // Debounce
    if (!mailboxOpen) {
      openMailbox();
      pushButtonPressCount++; // Increment push button press counter when mailbox is opened
      Serial.println("Mailbox opened by user.");
      Serial.print("Push button press count: ");
      Serial.println(pushButtonPressCount);
      voneClient.publishTelemetryData(PushButton,"Button1", pushButtonPressCount);
      mailCount = 0;  // Reset mail count after opening
      Serial.print("Mail count reset to: ");
      Serial.println(mailCount);
      voneClient.publishTelemetryData(MailCountSensor, "Obstacle", mailCount);  // Reset mail count on platform
    } else {
      closeMailbox();
      Serial.println("Mailbox closed by user.");
    }
    updateLEDs();
    delay(500);  // Additional debounce delay
  }

  delay(100);  // General loop delay
}

// Open mailbox
void openMailbox() {
  mailboxServo.write(90);  // Open mailbox
  mailboxOpen = true;
  digitalWrite(redLED, HIGH); // Turn on red LED when mailbox is open
  digitalWrite(yellowLED, LOW); // Turn off yellow LED when mailbox is open
}

// Close mailbox
void closeMailbox() {
  mailboxServo.write(0);  // Close mailbox
  mailboxOpen = false;
  digitalWrite(redLED, LOW); // Turn off red LED when mailbox is closed
  updateLEDs();
}

// Function to blink an LED
void blinkLED(int ledPin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200); // Blink duration
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}

// Function to update LED indicators
void updateLEDs() {
  if (!mailboxOpen) {
    if (mailPresent) {
      digitalWrite(yellowLED, HIGH); // Mail present, yellow LED on
    } else {
      digitalWrite(yellowLED, LOW); // No mail, yellow LED off
    }
  } else {
    digitalWrite(yellowLED, LOW); // Mailbox open, yellow LED off
  }
}
