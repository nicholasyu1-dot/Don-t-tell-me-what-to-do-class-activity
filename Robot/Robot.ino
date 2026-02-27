
// this progam will be able to solve a simple maze/course
#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

// ====== PIN CONSTANT VALUES ======

#define NUM_LEDS 2            // Number of LEDs on your board
#define PIN_RBGLED 4          // LED Pin
#define PWR_R 5               // Right Motor Power
#define PWR_L 6               // Left Motor Power
#define MTR_R 8               // Right Motor Control
#define MTR_L 7               // Left Motor Control
#define SERVO 10              // Servo Motor
#define MTR_ENABLE 3          // Motor Enable Pin
#define US_OUT 13             // Ultrasonic Sensor Input
#define US_IN 12              // Ultrasonic Sensor Output
#define LINE_L A2             // Left Line Tracker
#define LINE_C A1             // Center Line Tracker
#define LINE_R A0             // Right Line Tracker
#define BUTTON 2              // Push Button
#define GYRO 0x68             // Gyro Sensor Address



// ====== PROGRAM VARIABLES ======
int16_t gyroZ;                // Raw gyro Z-axis reading
float gyroZOffset = 0;        // Calibration offset
float currentAngle = 0;       // Current angle in degrees
unsigned long lastTime = 0;   // Last read time
CRGB leds[NUM_LEDS];          // Current LED Color values
Servo scanServo;              // Servo


int SPEED = 70;
int colour = 325;
int Serv_angle = 95;//starting agle, based on my robot

CRGB colours[6] = {CRGB::Red,CRGB::Blue,CRGB::Green,CRGB::Yellow,CRGB::Purple,CRGB::White};//all the colors neeed here



void setup() {
  

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






void loop() {
  int distance = getDistance();
  updateGyroAngle();

  if (distance<=5 && distance!=0){  digitalWrite(MTR_ENABLE, LOW);//stops at the end of garage/maze
}

  if (analogRead(LINE_L) < colour ){

    turn(180,0);//turns 90 deg(function takes the angle divided by 2) to the RIGHT
    
  
  }
  else if (analogRead(LINE_R  )< colour ){

    turn(180,1);//turns 90 deg(function takes the angle divided by 2) to the LEFT
  
  }
  else{

 
    adjustangleTo(0);
    //robot goes forward

    analogWrite(PWR_R,SPEED);
    analogWrite(PWR_L,SPEED);
    digitalWrite(MTR_R,HIGH);
    digitalWrite(MTR_L,HIGH);



  }
}

//code to adjust the robot back to the right angle, if it drifts too far
void adjustangleTo(float targetAngle) {
    const int minPower = 60;
    const int maxPower = 100;
    const float deadband = 2.0;

    while (true) {
        updateGyroAngle();

        float angleError = targetAngle - getAngle();
        if (angleError > 180) angleError -= 360;
        if (angleError < -180) angleError += 360;

        if (abs(angleError) <= deadband) break;  // aligned

        int power = constrain(abs(angleError) * 5, minPower, maxPower);

        if (angleError > 0) { // clockwise
            analogWrite(PWR_R, power);
            analogWrite(PWR_L, power);
            digitalWrite(MTR_R, HIGH);
            digitalWrite(MTR_L, LOW);
        } else {              // counter-clockwise
            analogWrite(PWR_R, power);
            analogWrite(PWR_L, power);
            digitalWrite(MTR_R, LOW);
            digitalWrite(MTR_L, HIGH);
        }
    }

    // stop motors
    analogWrite(PWR_R, 0);
    analogWrite(PWR_L, 0);
    digitalWrite(MTR_R, LOW);
    digitalWrite(MTR_L, LOW);
}


//side = 0 means right, side =1 means left
void turn(int angle, int side){
  analogWrite(PWR_R,0);
  analogWrite(PWR_L,0);

  Serial.println("side:");
  Serial.println(side);
  

  delay(600);
  resetAngle();
  //will turn until it reaches that degree.
  while (not(abs((getAngle()))>(angle/2-0.05) && abs((getAngle()))<(angle/2+0.05))){

  updateGyroAngle();
  Serial.println(abs((getAngle())));
  
  if (side == 1){
    digitalWrite(MTR_R,LOW);
    digitalWrite(MTR_L,HIGH);

     Serial.println("left");
     }

  if (side == 0){
    
    digitalWrite(MTR_R,HIGH);
    digitalWrite(MTR_L,LOW);

    Serial.println("RIGHT");



  }
    analogWrite(PWR_R,45);
    analogWrite(PWR_L,(45));


  }

    analogWrite(PWR_R,0);
    analogWrite(PWR_L,(0));
    resetAngle();
    delay(100);
  


}
// ====== LED FUNCTIONS ======

void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

// ===== SERVO FUNCTIONS =====

// Set servo angle (0–180 degrees)
void setServoAngle(int angle) {
  static int lastAngle = -1;
  angle = constrain(angle, 0, 180);

  if (angle != lastAngle) {
    scanServo.write(angle);
    delay(15);  // Allow servo to settle
    
    lastAngle = angle;

  }
}


// Center the servo
void centerServo() {
  setServoAngle(95);
}


// ====== GYRO FUNCTIONS ======

// Initialize Gyro Sensor
bool setupGyro() {



  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // Configure gyro sensitivity (±250 deg/s)
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250 deg/s
  Wire.endTransmission();
  
  lastTime = micros();
  return true;
}

// Calibrate gyro (robot must be stationary!)
void calibrateGyro() {
  delay(500);
  
  long sum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);  // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);
    
    int16_t gz = Wire.read() << 8 | Wire.read();
    
    sum += gz;
    delay(10);
  }
  
  gyroZOffset = sum / samples;
  currentAngle = 0;
}

// Read gyro Z-axis
int16_t readGyroZ() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);  // GYRO_ZOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  
  int16_t gz = Wire.read() << 8 | Wire.read();
  return gz;
}

// MUST be called frequently (e.g., every loop iteration)
// Angle accuracy degrades if this is not called often
void updateGyroAngle() {

  
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;  // Time in seconds
  lastTime = now;
  
  if (dt <= 0 || dt > 0.1) {
    lastTime = micros();
    return;
  }
  // Read gyro
  gyroZ = readGyroZ();
  
  // Convert to degrees per second (sensitivity = 131 for ±250 deg/s)
  // INVERTED THE SIGN HERE to fix direction!
  float gyroRate = -((gyroZ - gyroZOffset) / 131.0);

  currentAngle += gyroRate * dt;
  
  // Keep angle in range -180 to +180
  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;


  // Serial.print("dt: "); Serial.print(dt, 6);
  // Serial.print("  raw: "); Serial.print(gyroZ - gyroZOffset);
  // Serial.print("  rate: "); Serial.println(gyroRate, 4);
}

// Reset angle to zero
void resetAngle() {
  currentAngle = 0;
}

// Get current angle
float getAngle() {
  return currentAngle;
}

// ===== ULTRASONIC SENSOR FUNCTIONS =====

// Returns distance in centimeters, or 0 if invalid
int getDistance() {
  int validReading = 0;
  int attempts = 0;
  
  while (validReading == 0 && attempts < 3) {
    if (attempts > 0) delay(60);  // Only delay on retries
    
    digitalWrite(US_OUT, LOW);
    delayMicroseconds(2);
    digitalWrite(US_OUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_OUT, LOW);
    
    long duration = pulseIn(US_IN, HIGH, 30000);
    int distance = duration * 0.034 / 2;
    
    if (duration > 0 && distance <= 200) {
      validReading = distance;
    }
    
    attempts++;
  }


  return validReading;
}
