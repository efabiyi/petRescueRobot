// #include <Arduino.h> 
// #include "driver/ledc.h" 
// #include <ESP32Servo.h> 

// #define CLAW_PIN 21
// #define Y_PIN 20
// #define X_PIN 25 
// #define Z_PIN 10
// #define HALL_PIN 32 

// #define CLAW_CHAN 0 
// #define Y_CHAN 1 
// #define X_CHAN 2 
// #define Z_CHAN 3 

// #define BITS 16 
// #define FREQ 50 

// #define MG996R_MIN_US 500 
// #define MG996R_MAX_US 2500 

// #define _25KGCM_MIN_US 750 
// #define _25KGCM_MAX_US 2250 

// #define LINK1 8 
// #define LINK2 13 

// const float HALL_VOLTAGE_REF = 3.3; 
// const float MAGNET_THRESHOLD_VOLTAGE = 1; 

// const int CLAW_OPEN_ANGLE = 0; 
// const int CLAW_CLOSE_ANGLE = 110; 

// const int Z_STEP_SEARCH_ANGLE = 2; 
// const int Z_IDLE_ANGLE = 90; 

// const int Y_STEP_SEARCH = 5; 
// const int Y_IDLE_ANGLE = 110; 

// const int X_STEP_SEARCH = 5; 
// const int X_IDLE_ANGLE = 90; 

// int CURRENT_X_ANGLE = X_IDLE_ANGLE; 
// int CURRENT_Y_ANGLE = Y_IDLE_ANGLE; 
// int CURRENT_Z_ANGLE = Z_IDLE_ANGLE; 
// bool CLAW_IS_OPEN = true; 

// int BEST_X_ANGLE = X_IDLE_ANGLE; 
// int BEST_Y_ANGLE = Y_IDLE_ANGLE; 

// float DISTANCE = 14; 
// float Z_DEGREE = 90; 

// long _map(long x, long in_min, long in_max, long out_min, long out_max) { 
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
// } 

// #define GET_DUTY_MG996R(angle_val) \ 
//   ( (int)_map( \ 
//       _map( (angle_val) , 0, 180, MG996R_MIN_US, MG996R_MAX_US), \ 
//       0, (1000000L / FREQ), 0, ((1 << BITS) - 1) \ 
//     ) \ 
//   ) 

// #define GET_DUTY_25KGCM(angle_val) \ 
//   ( (int)_map( \ 
//       _map( (angle_val) , 0, 180, _25KGCM_MIN_US, _25KGCM_MAX_US), \ 
//       0, (1000000L / FREQ), 0, ((1 << BITS) - 1) \ 
//     ) \ 
//   ) 

// float readVoltage(int pin); 
// bool magnetDetected(float voltage); 
// void moveToHome(); 
// bool searchPet(); 
// float distanceFromOrigin(float x, float y); 
// float getShoulderAngle(float x, float y); 
// float getElbowAngle(float x, float y); 

// void setup() { 
//   Serial.begin(9600); 

//   ledcSetup(CLAW_CHAN, FREQ, BITS); 
//   ledcAttachPin(CLAW_PIN, CLAW_CHAN); 

//   ledcSetup(Y_CHAN, FREQ, BITS); 
//   ledcAttachPin(Y_PIN, Y_CHAN); 

//   ledcSetup(X_CHAN, FREQ, BITS); 
//   ledcAttachPin(X_PIN, X_CHAN); 

//   ledcSetup(Z_CHAN, FREQ, BITS); 
//   ledcAttachPin(Z_PIN, Z_CHAN); 

//   pinMode(HALL_PIN, INPUT); 
//   analogReadResolution(12); 

//   moveToHome(); 

//   delay(2000); 

//   Serial.println("Robot Arm at home potion. Ready to detect magnets."); 
// } 

// void loop() { 

//   bool petFound = searchPet(); 

//   if(petFound) { 
//     ledcWrite(X_CHAN, GET_DUTY_25KGCM(90)); 
//     delay(500); 
//     ledcWrite(Y_CHAN, GET_DUTY_25KGCM(90)); 
//     delay(500); 
//     ledcWrite(Z_CHAN, GET_DUTY_MG996R(90));
//     delay(500);
//     // ledcWrite(Z_CHAN, GET_DUTY_MG996R(180));
//     // delay(2000);
//     ledcWrite(CLAW_CHAN, GET_DUTY_MG996R(CLAW_OPEN_ANGLE)); 
//     delay(500); 

//     moveToHome(); 
//   }

//   moveToHome();
   

//   int rawSensorValue = analogRead(HALL_PIN); // Read raw ADC value

//   float hallVoltage = readVoltage(HALL_PIN); 
//   bool petLocated = magnetDetected(hallVoltage); 
   
//   String hallData = "Raw ADC: " + String(rawSensorValue) + " Z_Angle: " + String(CURRENT_Z_ANGLE) + " | Hall Sensor Voltage: " + String(hallVoltage, 3) + " V, Pet Located: " + (petLocated ? "Yes" : "No"); 
//   Serial.println(hallData); 
// } 

// float readVoltage(int pin) { 
//   int sensorValue = analogRead(pin); 
//   float voltage = sensorValue * (HALL_VOLTAGE_REF / 4095.0); 
//   return voltage; 
// } 

// bool magnetDetected(float voltage) { 
//   if (voltage > MAGNET_THRESHOLD_VOLTAGE) { 
//     return true; 
//   } 
//   return false; 
// } 

// void moveToHome() { 
//   Serial.println("Moving arm to home position..."); 
//   ledcWrite(CLAW_CHAN, GET_DUTY_MG996R(CLAW_CLOSE_ANGLE)); 
//   ledcWrite(X_CHAN, GET_DUTY_25KGCM(X_IDLE_ANGLE)); 
//   ledcWrite(Y_CHAN, GET_DUTY_25KGCM(Y_IDLE_ANGLE)); 
//   ledcWrite(Z_CHAN, GET_DUTY_MG996R(Z_IDLE_ANGLE)); 

//   CLAW_IS_OPEN = false; 
//   CURRENT_X_ANGLE = X_IDLE_ANGLE; 
//   CURRENT_Y_ANGLE = Y_IDLE_ANGLE; 
//   CURRENT_Z_ANGLE = Z_IDLE_ANGLE; 
//   delay(500); 
// } 

// float getElbowAngle(float x, float y) { 
//   float d = distanceFromOrigin(x, y); 
//   if (d > LINK1 + LINK2) return -1; 
//   return degrees(acos(((LINK1 * LINK1) + (LINK2 * LINK2) - d * d) / (2 * LINK1 * LINK2))); 
// } 

// float getShoulderAngle(float x, float y) { 
//   float d = distanceFromOrigin(x, y); 
//   if (d > LINK1 + LINK2) return -1; 
   
//   float beta = degrees(acos(((LINK1 * LINK1) + (d * d) - (LINK2 * LINK2)) / (2 * LINK1 * d))); 
//   float alpha = degrees(atan2(y,x)); 

//   float shoulderAngle = beta + alpha; 

//   return shoulderAngle; 
// } 

// float distanceFromOrigin(float x, float y) { 
//   return sqrt((x * x) + (y * y)); 
// } 

// bool searchPet() { 
//   bool petFound = false; 
//   float maxVoltage = HALL_VOLTAGE_REF; 

//   float Z_OFFSET =  Z_DEGREE;

//   ledcWrite(Z_CHAN, GET_DUTY_MG996R(Z_OFFSET)); 

//   for (float y = 10; y >= 0; y -= 0.5) { 
//     float elbowDeg = getElbowAngle(DISTANCE, y); 
//     float shoulderDeg = getShoulderAngle(DISTANCE, y); 

//     if (elbowDeg < 0 || shoulderDeg < 0) continue; 

//     int shoulderServoAngle = shoulderDeg; 
//     int elbowServoAngle = elbowDeg; 

//     ledcWrite(X_CHAN, GET_DUTY_25KGCM(shoulderServoAngle)); 
//     ledcWrite(Y_CHAN, GET_DUTY_25KGCM(elbowServoAngle)); 
//     delay(300); 

//     CURRENT_X_ANGLE = shoulderServoAngle; 
//     CURRENT_Y_ANGLE = elbowServoAngle; 

//     float currentVoltage = readVoltage(HALL_PIN); 

//     Serial.print("x: "); 
//     Serial.print(DISTANCE); 
//     Serial.print(" y: "); 
//     Serial.print(y); 
//     Serial.print(" Shoulder: "); 
//     Serial.print(shoulderServoAngle); 
//     Serial.print(" Elbow: "); 
//     Serial.print(elbowServoAngle); 
//     Serial.print(" Voltage: "); 
//     Serial.println(currentVoltage); 

//     if (currentVoltage < maxVoltage) { 
//       maxVoltage = currentVoltage; 
//       BEST_X_ANGLE = shoulderServoAngle; 
//       BEST_Y_ANGLE = elbowServoAngle; 
//     } 

//     if (magnetDetected(currentVoltage)) { 
//       Serial.println("Pet Detected!"); 
//       ledcWrite(Y_CHAN, GET_DUTY_25KGCM(Y_IDLE_ANGLE)); 
//       delay(500); 
//       ledcWrite(Z_CHAN, GET_DUTY_MG996R(Z_OFFSET)); 
//       delay(500); 
//       ledcWrite(CLAW_CHAN, GET_DUTY_MG996R(CLAW_OPEN_ANGLE)); 
//       delay(500);
//       ledcWrite(Y_CHAN, GET_DUTY_25KGCM(elbowServoAngle + 5)); 
//       delay(500); 
//       ledcWrite(X_CHAN,GET_DUTY_25KGCM(shoulderServoAngle - 15)); 
//       ledcWrite(CLAW_CHAN, GET_DUTY_MG996R(CLAW_CLOSE_ANGLE)); 
//       delay(500); 
//       CLAW_IS_OPEN = false; 
//       return true; 
//     } 
//   } 

//   if (maxVoltage > HALL_VOLTAGE_REF) { 
//     ledcWrite(X_CHAN, GET_DUTY_25KGCM(BEST_X_ANGLE)); 
//     delay(300); 
//     ledcWrite(Y_CHAN, GET_DUTY_25KGCM(BEST_Y_ANGLE)); 
//     delay(300); 
//     return (maxVoltage > MAGNET_THRESHOLD_VOLTAGE); 
//   } 

//   return false; 
// }