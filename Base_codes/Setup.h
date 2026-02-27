#include <Wire.h>
#include "Sensors.h"
#include "Others.h"
#include "Motors.h"

             // Left Motor Control
          // Right Line Tracker
#define BUTTON 2              // Push Button



void start() {
  

  // setup LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // 0-255
  
  // Motor pins
  pinMode(PWR_R, OUTPUT);
  pinMode(PWR_L, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  // Line tracking sensor pins
  pinMode(LINE_L, INPUT);
  pinMode(LINE_C, INPUT);
  pinMode(LINE_R, INPUT);
  
  // Button pin
  pinMode(BUTTON, INPUT_PULLUP);

  // Enable motor driver
  digitalWrite(MTR_ENABLE, HIGH);

  // Initialize serial communication
  Serial.begin(115200);

  // Initialize Servo motor
  scanServo.attach(SERVO);
  centerServo();   // Center position
  
  
  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {

  }

  delay(500);

  // Initialize Gyro - hard stop if failed
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true);  // Hard stop
  }

  calibrateGyro();
  
  digitalWrite(MTR_R,HIGH);//enables motors
  digitalWrite(MTR_L,HIGH);
  
}