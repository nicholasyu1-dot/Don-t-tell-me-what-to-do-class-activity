
#define PWR_R 5               // Right Motor Power
#define PWR_L 6               // Left Motor Power
#define MTR_R 8               // Right Motor Control
#define MTR_L 7  
#define MTR_ENABLE 3          // Motor Enable Pin

void forward(int SPEED){
    analogWrite(PWR_R,SPEED);
    analogWrite(PWR_L,SPEED);
    digitalWrite(MTR_R,HIGH);
    digitalWrite(MTR_L,HIGH);}

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