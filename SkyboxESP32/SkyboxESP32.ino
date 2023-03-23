#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

BluetoothSerial SerialBT;

// Pin definitions
#define pwmPin 4 // PWM-capable GPIO pin
#define PIN_IN1  27 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PIN_IN2  15 // ESP32 pin GIOP15 connected to the IN2 pin L298N
#define extendLimit 26  //Limit switch for fully extended DLP
#define retractLimit 25 //Limit switch for fully retracted DLP
#define startButton 14  //Button on skybox to start deployment
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C instantiation for weather station

// PWM properties
const int freq = 10000; // Frequency in Hz
const int motorChannel = 0; // PWM channel
const int resolution = 8; // Resolution in bits (e.g., 8-bit resolution = 0-255)

// Motor speed
const int motorSpeed = 255; // Set a value between 0 (stopped) and 255 (full speed)
bool extending = false;
bool retracting = false;

bool asked_user = false; // Keep track of whether the user has been asked the question yet

void setup() {
  // Pin modes
  pinMode(pwmPin, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(extendLimit, INPUT_PULLUP);
  pinMode(retractLimit, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);
  attachInterrupt(startButton, button_isr, FALLING);//Attach interruput to the button

  //------------------------------------------------
  //Code for BME280
  unsigned status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
  //-----------------------------------------------
  // Configure the PWM pin
  ledcSetup(motorChannel, freq, resolution);
  ledcAttachPin(pwmPin, motorChannel);

  // Set the motor speed
  ledcWrite(motorChannel, motorSpeed);

  // Direction pins
  digitalWrite(PIN_IN1, LOW); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, LOW);  // control the motor's direction in clockwise

  // Bluetooth
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  // if(!digitalRead(extendLimit)){
  //   SerialBT.println("Extend limit switch pressed");
  // }
  // if(!digitalRead(retractLimit)){
  //   SerialBT.println("Retract limit switch pressed");
  // }

  // if((!digitalRead(extendLimit) && extending) || (!digitalRead(retractLimit) && retracting)){//Stop if either limit switch triggers
  //   SerialBT.println("Limit Reached");
  //   stop();
  // }
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
}

//When the button is pressed and is fully retracted
void ARDUINO_ISR_ATTR button_isr() {
  SerialBT.println("Start button pressed");
  if(!digitalRead(extendLimit)){
    SerialBT.println("Extend limit pressed and button pressed");
    retract();
  }else if(!digitalRead(retractLimit)){
    SerialBT.println("Retract limit pressed and button pressed");
    extend();
  }
}

void extend(){
  digitalWrite(PIN_IN1, LOW); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, HIGH);  // control the motor's direction in clockwise
  extending = true;
  retracting = false;
}

void retract(){
  digitalWrite(PIN_IN1, HIGH); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, LOW);  // control the motor's direction in clockwise
  extending = false;
  retracting = true;
}

void stop(){
  digitalWrite(PIN_IN1, LOW); // control the motor's direction in clockwise
  digitalWrite(PIN_IN2, LOW);  // control the motor's direction in clockwise
}

bool flightChecks(){//This function will be used to return a boolean on whether it is acceptable for the drone to take off
  Serial.println("Begin flight checks");

  if(digitalRead(extendLimit)){//Is the door not fully extended?
  Serial.println("DLP not fully extended!");
    return false;
  }else{
    Serial.println("DLP fully extended");
  }

  if(bme.readTemperature() < 10){//Is the temperature not above 10 degrees C?
    Serial.println("Temperature too low!");
    return false;
  }else{
    Serial.print("Temp (C): ");
    Serial.println(bme.readTemperature());
  }

  /**
  Add flight check for wind speed
  */

  /**
  Add flight check for rain
  */

  Serial.println("All systems are go!");
  return true;
}
