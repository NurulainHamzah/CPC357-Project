#include "VOneMqttClient.h"  // Include MQTT library for cloud communication
#include <Adafruit_NeoPixel.h>  // Include library for NeoPixel control

#define NUMPIXELS 1  // Define number of pixels for the NeoPixel LED
#define DELAYVAL 500  // Delay time for LED blinking
#define INTERVAL 1000  // Time interval for publishing telemetry data

// Define device IDs for various IoT components
const char* Relay = "dbc79d27-03e5-4e08-87fd-a7a3f71269c6";      
const char* LEDLight1 = "0433e9c2-5328-4cc5-9b89-34af57769c92";  
const char* LEDLight2 = "49e3e128-d01b-4632-8804-b6f6bf9b4c17";  
const char* PIRSensor = "5a56eb25-088d-41e8-b0c3-7331f1956d98";  
const char* Button = "3ccbeba7-a56d-451e-8d72-17380d3bd3ed";     

// Define pins for the hardware components
const int relayPin = 14;        
const int ledPinGreen = 48;     
const int ledPinRed = 21;       
const int buttonPin = 47;       
const int motionSensorPin = 4;  
const int neoPin = 46;          

// Define variables for PIR sensor and button state
bool PIRvalue = false;
bool buttonState = false;
bool lastButtonState = false;
unsigned long lastMsgTime = 0;

// Create NeoPixel instance for onboard LED
Adafruit_NeoPixel pixels(NUMPIXELS, neoPin, NEO_GRB + NEO_KHZ800);

// Initialize VOneMqttClient instance for cloud communication
VOneMqttClient voneClient;

// Interrupt Service Routine (ISR) for detecting motion via PIR sensor
void IRAM_ATTR detectsMovement() {
  PIRvalue = true;  // Set PIR value to true when motion is detected
}

// Callback for actuator commands (Relay and LEDs)
void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand) {
  // Parse incoming commands and control corresponding actuators (fan, LEDs)
  JSONVar commandObjct = JSON.parse(actuatorCommand);
  JSONVar keys = commandObjct.keys();
  bool commandValue;

  for (int i = 0; i < keys.length(); i++) {
    String key = (const char*)keys[i];
    commandValue = (bool)commandObjct[keys[i]];
    
    // Control relay (fan) based on received command
    if (String(actuatorDeviceId) == Relay) {
      digitalWrite(relayPin, commandValue ? HIGH : LOW);
    }
    // Control LED lights based on received command
    if (String(actuatorDeviceId) == LEDLight2) {
      digitalWrite(ledPinGreen, commandValue ? HIGH : LOW);
    }
    if (String(actuatorDeviceId) == LEDLight1) {
      digitalWrite(ledPinRed, commandValue ? HIGH : LOW);
    }
  }
}

// WiFi setup for connecting to the network
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // Initialize serial communication and connect to Wi-Fi
  Serial.begin(115200);
  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);

  // Initialize pin modes for hardware components
  pinMode(relayPin, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinRed, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(motionSensorPin, INPUT);

  // Initialize Neopixel and attach interrupt for PIR sensor
  pixels.begin();
  pixels.show();
  attachInterrupt(digitalPinToInterrupt(motionSensorPin), detectsMovement, RISING);
}

void loop() {
  // Handle button press for triggering intruder alert
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      digitalWrite(ledPinRed, HIGH);  // Turn on red LED for intruder alert
      delay(10000);  // Keep LED on for 10 seconds
      digitalWrite(ledPinRed, LOW);   // Turn off red LED
    }
    lastButtonState = buttonState;
  }

  // Handle PIR sensor motion detection and trigger actions
  if (PIRvalue) {
    digitalWrite(ledPinRed, HIGH);  // Turn on red LED for intruder alert
    delay(5000);  // Keep LED on for 5 seconds
    PIRvalue = false;  // Reset PIR sensor state
  } else {
    digitalWrite(ledPinRed, LOW);
  }

  // Periodically publish telemetry data to VOne Cloud
  unsigned long cur = millis();
  if (cur - lastMsgTime > INTERVAL) {
    lastMsgTime = cur;

    // Prepare telemetry data to be published
    String telemetryData = "{";
    telemetryData += "\"button\": " + String(digitalRead(buttonPin)) + ", ";
    telemetryData += "\"PIR\": " + String(digitalRead(motionSensorPin)) + ", ";
    telemetryData += "\"LEDGreen\": " + String(digitalRead(ledPinGreen)) + ", ";
    telemetryData += "\"LEDRed\": " + String(digitalRead(ledPinRed)) + "}";
    
    // Publish telemetry data to VOne Cloud platform
    voneClient.publishDeviceStatusEvent(Button, telemetryData.c_str());
    voneClient.publishDeviceStatusEvent(PIRSensor, telemetryData.c_str());
  }

  // Ensure the connection with VOne Cloud is maintained
  if (!voneClient.connected()) {
    voneClient.reconnect();
  }
  voneClient.loop();  // Keep MQTT loop running
}