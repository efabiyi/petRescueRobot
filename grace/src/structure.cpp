// #include <Arduino.h>

// //-------------------------
// // ENUMS AND STRUCTURES
// //-------------------------

// enum State {
//   DRIVE,
//   ALIGN,
//   GRAB,
//   ESCAPE
// };

// struct Data {
//   int leftIRSensor;
//   int middleIRSensor;
//   int rightIRSensor;
//   ScanResult scanResult;
//   int hallSensor;
//   unsigned long currentTime;
//   int clawSensor;
//   // more inputs TBD
// };

// //-------------------------
// // GLOBAL STATE VARIABLES
// //-------------------------

// State currentState;
// State prevState;

// Data currentData;
// Data prevData;

// //-------------------------
// // COMPONENT PIN ASSIGNMENTS
// //-------------------------

// // Line Following Sensors (Analog Inputs)
// const int IR_LEFT    = 34;
// const int IR_MIDDLE  = 35;
// const int IR_RIGHT   = 36;

// // Claw Sensors (Digital Inputs)
// const int CLAW_SENSOR_1 = 32;
// const int CLAW_SENSOR_2 = 33;
// const int CLAW_SENSOR_3 = 23;

// // Basket and Lift Sensors (Digital Inputs)
// const int BASKET_SENSOR = 26;
// const int LIFT_SENSOR   = 27;

// // Pet Detection
// const int HALL_SENSOR = 39;
// const int I2C_SDA     = 21;
// const int I2C_SCL     = 22;

// // PWM Outputs for Motion and Manipulation
// const int LEFT_WHEEL_PWM  = 4;
// const int RIGHT_WHEEL_PWM = 5;

// const int SERVO_ROTATE_Z    = 13;
// const int SERVO_FORWARD_BACK = 14;
// const int SERVO_UP_DOWN      = 18;
// const int SERVO_CLAW         = 19;

// const int PULLEY_MOTOR = 25;

// // Free Digital Pins
// const int FREE_0 = 0;
// const int FREE_1 = 1;
// const int FREE_2 = 2;

// //-------------------------
// // INITIATION FUNCTIONS
// //-------------------------

// void initiateSensors() {
//   // Initialize IR sensors
//   pinMode(IR_LEFT, INPUT);
//   pinMode(IR_MIDDLE, INPUT);
//   pinMode(IR_RIGHT, INPUT);

//   // Initialize Claw sensors
//   pinMode(CLAW_SENSOR_1, INPUT);
//   pinMode(CLAW_SENSOR_2, INPUT);
//   pinMode(CLAW_SENSOR_3, INPUT);

//   // Initialize Basket and Lift sensors
//   pinMode(BASKET_SENSOR, INPUT);
//   pinMode(LIFT_SENSOR, INPUT);

//   // Initialize Hall sensor
//   pinMode(HALL_SENSOR, INPUT);

//   // Laser module on I2C — typically initialized via I2C library elsewhere
//   // Wire.begin(I2C_SDA, I2C_SCL); — done in setup if needed
// }

// void initiateClaw() {
//   // Configure servo pins for claw operation
//   pinMode(SERVO_CLAW, OUTPUT);
//   pinMode(SERVO_ROTATE_Z, OUTPUT);
//   pinMode(SERVO_FORWARD_BACK, OUTPUT);
//   pinMode(SERVO_UP_DOWN, OUTPUT);
// }

// void initiateMotors() {
//   // Configure motor pins for locomotion and lifting
//   pinMode(LEFT_WHEEL_PWM, OUTPUT);
//   pinMode(RIGHT_WHEEL_PWM, OUTPUT);
//   pinMode(PULLEY_MOTOR, OUTPUT);
// }

// //-------------------------
// // SENSOR READING FUNCTION
// //-------------------------

// Data readData() {
//   Data d;
//   d.leftIRSensor = analogRead(IR_LEFT);
//   d.middleIRSensor = analogRead(IR_MIDDLE);
//   d.rightIRSensor = analogRead(IR_RIGHT);
//   d.hallSensor = analogRead(HALL_SENSOR);
//   d.currentTime = millis();
//   // Placeholder for claw and beacon inputs
//   d.clawSensor = digitalRead(CLAW_SENSOR_1); // simplified for now
//   d.IRSensor = 0; // depends on laser or beacon implementation
//   return d;
// }

// //-------------------------
// // STATE LOGIC HANDLER
// //-------------------------

// State getState() {
//   // Logic to transition between states based on `currentData`
//   return DRIVE; // Placeholder
// }

// //-------------------------
// // STATE HANDLING FUNCTIONS
// //-------------------------

// void handleDrive() {
//   // Uses: IR sensors, Left/Right Wheel PWM
// }

// void handleLocatePet() {
//   // Uses: Hall sensor, potentially I2C Laser module
// }

// void handleGrabPet() {
//   // Uses: Servo Arm (Rotate, Forward/Back, Up/Down), Claw
// }

// void handleStorePet() {
//   // Uses: Pulley motor, Basket/Lift sensors
// }

// void handleEscape() {
//   // Uses: Likely same components as DRIVE, maybe more
// }

// //-------------------------
// // MAIN PROGRAM
// //-------------------------

// void setup() {
//   initiateSensors();
//   initiateClaw();
//   initiateMotors();

//   // Optionally initialize serial or I2C here
//   // Serial.begin(9600);
//   // Wire.begin(I2C_SDA, I2C_SCL);
// }

// void loop() {
//   currentData = readData();
//   currentState = getState();
  
//   switch(currentState) {
//     case DRIVE:
//       drive();
//       break;
//     case ALIGN:
//       handleLocatePet();
//       break;
//     case GRAB:
//       handleGrabPet();
//       break;
//     case ESCAPE:
//       handleStorePet();
//       break;
//   }

//   prevData = currentData;
// }
