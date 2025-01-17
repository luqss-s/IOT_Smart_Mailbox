#include "VOneMqttClient.h"
#include <ESP32Servo.h>

const char* InfraredSensor = "7b1bf7f0-db4a-423e-9049-58e5e510a950";  // Sensor for postman
const char* MailCountSensor = "b0c7a297-13d0-4325-8548-3990308fdf56";  // Sensor for mail 
const char* PushButton = "5f69e5e2-be5c-4806-a182-abfae609a144";  // Push button
const char* ServoMotor = "d30df230-ab7b-46cf-a6a4-9e3ee5bec8f5";  // Servo motor

// Pin definitions
const int irDeliverySensorPin = 4;  // IR Sensor to detect postman
const int irInsideSensorPin = 42;   // IR Sensor to detect mail inside the box
const int greenLED = 5;             // Green LED: 
const int yellowLED = 6;            // Yellow LED: 
const int redLED = 9;               // Red LED: 
const int buttonPin = 38;           // Push button
const int servoPin = 8;             // Servo motor pin

VOneMqttClient voneClient;
Servo mailboxServo;

bool mailPresent = false;
bool postmanDetected = false;
int mailCount = 0;
int postmanCount = 0;  
int pushButtonPressCount = 0; 
bool mailboxOpen = false;

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

void triggerServo_callback(const char* actuatorDeviceId, const char* actuatorCommand)
{
  Serial.print("Main received callback : ");
  Serial.print(actuatorDeviceId);
  Serial.print(" : ");
  Serial.println(actuatorCommand);

  String errorMsg = "";

  JSONVar commandObjct = JSON.parse(actuatorCommand);
  JSONVar keys = commandObjct.keys();

  if (String(actuatorDeviceId) == ServoMotor)
  {
    String key = "";
    JSONVar commandValue = "";
    for (int i = 0; i < keys.length(); i++) {
      key = (const char* )keys[i];
      commandValue = commandObjct[keys[i]];
    }
    
    Serial.print("Key : ");
    Serial.println(key.c_str());
    Serial.print("value : ");
    Serial.println(commandValue);

    int angle = (int)commandValue;
    mailboxServo.write(angle); 

    if (angle == 90) {
      mailboxOpen = true;
      mailCount = 0;  
      Serial.print("Mailbox opened by servo. Mail count reset to: ");
      Serial.println(mailCount);
      voneClient.publishTelemetryData(MailCountSensor, "MailCount", mailCount); 
      pushButtonPressCount++;
      voneClient.publishTelemetryData(PushButton, "Collect", pushButtonPressCount);
    } else {
      mailboxOpen = false;  
    }
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true); // Publish actuator status
  }
  else
  {
    Serial.print(" No actuator found : ");
    Serial.println(actuatorDeviceId);
    errorMsg = "No actuator found";
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false); // Publish error
  }
}

void setup() {
  Serial.begin(9600);
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
  mailboxServo.write(0);

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

  // Handle push button for manual control (always active)
  if (digitalRead(buttonPin) == LOW) {
    delay(200); 
    if (!mailboxOpen) {
      openMailbox();  // Open mailbox
      pushButtonPressCount++;
      Serial.println("Mailbox opened by user.");
      Serial.print("Push button press count: ");
      Serial.println(pushButtonPressCount);
      voneClient.publishTelemetryData(PushButton, "Collect", pushButtonPressCount);
      
      // Reset mail count after opening
      mailCount = 0;
      Serial.print("Mail count reset to: ");
      Serial.println(mailCount);
      voneClient.publishTelemetryData(MailCountSensor, "MailCount", mailCount);
    } else {
      closeMailbox(); 
      Serial.println("Mailbox closed by user.");
    }
    updateLEDs();
    delay(500); 
  }
  // Skip sensor detection if the mailbox is open
  if (mailboxOpen) {
    return; 
  }

  // Detect postman delivery
  if (digitalRead(irDeliverySensorPin) == HIGH) {
    if (!postmanDetected) {
      postmanDetected = true;
      postmanCount++;
      Serial.println("Postman detected! Checking for delivery...");
      Serial.print("Postman count: ");
      Serial.println(postmanCount);
      blinkLED(greenLED, 1); 
      voneClient.publishTelemetryData(InfraredSensor, "PostmanCount", postmanCount);
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
      blinkLED(yellowLED, 1);
      voneClient.publishTelemetryData(MailCountSensor, "MailCount", mailCount);
      updateLEDs();
    }
  } else {
    if (mailPresent) {
      mailPresent = false;
      updateLEDs();
    }
  }
  delay(100); 
}

// Open mailbox
void openMailbox() {
  int angle = 90;
  mailboxServo.write(angle);  
  mailboxOpen = true;
  digitalWrite(redLED, HIGH); 
  digitalWrite(yellowLED, LOW);

  // Publish actuator status event
  String errorMsg = "";
  String actuatorCommand = "{\"Servo\":" + String(angle) + "}";  
  voneClient.publishActuatorStatusEvent(ServoMotor, actuatorCommand.c_str(), errorMsg.c_str(), true);
}

// Close mailbox
void closeMailbox() {
  int angle = 0;
  mailboxServo.write(angle);  
  mailboxOpen = false;
  digitalWrite(redLED, LOW); 
  updateLEDs();

  String errorMsg = "";
  String actuatorCommand = "{\"Servo\":" + String(angle) + "}";  
  voneClient.publishActuatorStatusEvent(ServoMotor, actuatorCommand.c_str(), errorMsg.c_str(), true);
}

// Function to blink an LED
void blinkLED(int ledPin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200); 
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}

// Function to update LED indicators
void updateLEDs() {
  if (!mailboxOpen) {
    if (mailPresent) {
      digitalWrite(yellowLED, HIGH); 
    } else {
      digitalWrite(yellowLED, LOW); 
    }
  } else {
    digitalWrite(yellowLED, LOW); 
  }
}
