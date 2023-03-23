#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// Pin definitions
#define pwmPin 4 // PWM-capable GPIO pin
#define PIN_IN1  27 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PIN_IN2  15 // ESP32 pin GIOP15 connected to the IN2 pin L298N
#define extendLimit 26  //Limit switch for fully extended DLP
#define retractLimit 13 //Limit switch for fully retracted DLP
#define startButton 14  //Button on skybox to start deployment

// PWM properties
const int freq = 10000; // Frequency in Hz
const int motorChannel = 0; // PWM channel
const int resolution = 8; // Resolution in bits (e.g., 8-bit resolution = 0-255)

// Motor speed
const int motorSpeed = 255; // Set a value between 0 (stopped) and 255 (full speed)
bool moving = false;

bool asked_user = false; // Keep track of whether the user has been asked the question yet

void setup() {
  // Pin modes
  pinMode(pwmPin, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(extendLimit, INPUT_PULLUP);
  pinMode(retractLimit, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);
  attachInterrupt(startButton, button_isr, FALLING);

  
  // Configure the PWM pin
  ledcSetup(motorChannel, freq, resolution);
  ledcAttachPin(pwmPin, motorChannel);

  // Set the motor speed
  ledcWrite(motorChannel, motorSpeed);

  // Direction pins
  digitalWrite(PIN_IN1, LOW); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, HIGH);  // control the motor's direction in clockwise

  // Bluetooth
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if(!digitalRead(extendLimit)){
    SerialBT.println("Extended limit switch pressed");
  }
  if(!digitalRead(retractLimit)){
    SerialBT.println("Retracted limit switch pressed");
  }
  // Bluetooth code
  if (SerialBT.connected(115200)) {
    if (!asked_user) {
      SerialBT.println("'in' or 'out'? (i/o)");
      asked_user = true; // Set asked_user to true to indicate that the user has been asked
    }
    if (SerialBT.available() > 0) { // Check if user input is available
      char response = SerialBT.read(); // Read the user input
      if (response == 'i') {
        SerialBT.println("moving...");
        extend();
        //SerialBT.println("done.");
        asked_user = false; // Set asked_user to false to indicate that the user has responded
      } else if (response == 'o') {
        SerialBT.println("moving...");
        retract();
        //SerialBT.println("done.");
        asked_user = false; // Set asked_user to false to indicate that the user has responded
      }
    }
  }

  // Motor code
  
  delay(500); // wait for half a second before repeating
}

void ARDUINO_ISR_ATTR button_isr() {
  SerialBT.println("Start button pressed");
  //delay(10);//debounce
}

void extend(){
  digitalWrite(PIN_IN1, LOW); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, HIGH);  // control the motor's direction in clockwise

}

void retract(){
  digitalWrite(PIN_IN1, HIGH); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, LOW);  // control the motor's direction in clockwise

}

void stop(){
  digitalWrite(PIN_IN1, LOW); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, LOW);  // control the motor's direction in clockwise
}
