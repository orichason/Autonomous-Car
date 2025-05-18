#include <ESP32Servo.h>
#include <ps5Controller.h>

Servo servo;
Servo esc;
const int servoPin = 5; // Pin number for the servo
const int escPin = 4; // Pin number for the ESC

void setServo(uint16_t microseconds) {
  //Serial.println("setServo called");
  servo.writeMicroseconds(microseconds);
}

void setESC(uint16_t microseconds) {
  //Serial.println("setESC called");
  esc.writeMicroseconds(microseconds);
}

bool getInstruction(){ 
    if(Serial.available() > 0){
      char device = Serial.read();
      if(device == 's' || device == 'e'){
        Serial.println("Received " + String(device) + " command");  // Send acknowledgment to Python

        // Wait for the remaining 3 bytes
        while (Serial.available() < 3) {
          // Wait for data to arrive
        }
        byte servoBuffer[3];
        Serial.readBytes(servoBuffer, 3);
        
        uint16_t microseconds = (servoBuffer[1] << 8) | servoBuffer[0];
        uint8_t receivedChecksum = servoBuffer[2];
        uint8_t calculatedChecksum = (servoBuffer[0] + servoBuffer[1]) & 0xFF;

        if(receivedChecksum == calculatedChecksum){
           if (device == 's') {
            setServo(microseconds);
            Serial.println("Servo moved to " + String(microseconds));
           } 
           else if (device == 'e') {
            setESC(microseconds);
            Serial.println("ESC set to " + String(microseconds));
           }
          return true;
        }
        else {
          Serial.println("Error: Checksum mismatch");
        }
      }
      else {
        Serial.println("Error: Unknown device identifier");
      }
    }
  return false;
}

void handShake(){
  while (Serial.available() > 0){
    Serial.read();
  }
  
  bool handShakeComplete = false;
  unsigned long startTime = millis();

  while(!handShakeComplete){
    if(Serial.available() >= 5){
      char receivedData[6];
      Serial.readBytes(receivedData, 5);
      receivedData[5] = '\0';

      if(strcmp(receivedData, "ready") == 0){
        //Serial.println("Arduino: Received handshake message from PC");
        Serial.write("ready");
        handShakeComplete = true;
        //Serial.println("Arduino: Handshake complete");
      }
      else{
//        Serial.print("Arduino: Unexpected message -> ");
//        Serial.println(receivedData);
      }
      
    }
    if (millis() - startTime > 5000) {  // 5-second timeout
      break;
    }
  }
}

int getSteering(){
  int steer_raw = ps5.LStickX();  // -128 to 127
  int steer_us;

  if (steer_raw < -10) {
    // Moved left
    steer_us = map(steer_raw, -128, 0, 1000, 1500); // scale to left PWM
  } else if (steer_raw > 10) {
    // Moved right
    steer_us = map(steer_raw, 0, 127, 1500, 2000);  // scale to right PWM
  } else {
    // Center
    steer_us = 1500;
  }
  
  return steer_us;
}

int getThrottle(){
  if(ps5.R2Value() > 0 && ps5.L2Value() == 0){
    // Forward throttle input
    return map(ps5.R2Value(), 0, 255, 1500, 2000); 
  }
  else if(ps5.L2Value() > 0 && ps5.R2Value() == 0){
    // Backward throttle input
    return map(ps5.L2Value(), 0, 255, 1500, 1000);
  }
  else{
    // No throttle input
    return 1500; // Neutral position for ESC
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){
    //Waiting for serial port to connect
  }
  servo.setPeriodHertz(50); // Set the frequency to 50Hz (20ms period)
  esc.setPeriodHertz(50); // Set the frequency to 50Hz (20ms period)
  esc.attach(escPin);
  delay(1000); // Give some time for the servo to initialize
  servo.attach(servoPin);

  ps5.begin("4C:B9:9B:A9:63:16");

  //handShake();
  while (!ps5.isConnected()) {
    Serial.println(" Not Connected");

  }
  Serial.println("Connected!");

}

void loop() {
  // put your main code here, to run repeatedly:
  //getInstruction();

  //getInstruction();  
  //setServo(getSteering());
  int throttle = getThrottle();
  int steering = getSteering();
  //setESC(throttle);
  setServo(steering);
}