// #include <Arduino.h>
// #include <Wire.h>
// #include "sensors.h"
// #include "state.h"

// const int IR_LEFT    = 34;
// const int IR_MIDDLE  = 35;
// const int IR_RIGHT   = 36;
// const int CLAW_SENSOR_1 = 32;
// const int CLAW_SENSOR_2 = 33;
// const int CLAW_SENSOR_3 = 23;
// const int BASKET_SENSOR = 26;
// const int LIFT_SENSOR   = 27;
// const int HALL_SENSOR = 39;
// const int I2C_SDA     = 21;
// const int I2C_SCL     = 22;

// void initiateSensors() {
//     pinMode(IR_LEFT, INPUT);
//     pinMode(IR_MIDDLE, INPUT);
//     pinMode(IR_RIGHT, INPUT);
//     pinMode(CLAW_SENSOR_1, INPUT);
//     pinMode(CLAW_SENSOR_2, INPUT);
//     pinMode(CLAW_SENSOR_3, INPUT);
//     pinMode(BASKET_SENSOR, INPUT);
//     pinMode(LIFT_SENSOR, INPUT);
//     pinMode(HALL_SENSOR, INPUT);
// }

// void initiateI2C() {
//     Wire.begin(I2C_SDA, I2C_SCL);
// }

// int readIRSensor(int pin) {
//     return analogRead(pin);
// }

// int readClawSensor(int sensorNum) {
//     switch(sensorNum) {
//         case 1: return digitalRead(CLAW_SENSOR_1);
//         case 2: return digitalRead(CLAW_SENSOR_2);
//         case 3: return digitalRead(CLAW_SENSOR_3);
//         default: return -1;
//     }
// }

// int readHallSensor() {
//     return analogRead(HALL_SENSOR);
// }

// bool readBasketSensor() {
//     return digitalRead(BASKET_SENSOR);
// }

// bool readLiftSensor() {
//     return digitalRead(LIFT_SENSOR);
// }

// Data readData() {
//     Data d;
//     d.leftIRSensor = readIRSensor(IR_LEFT);
//     d.middleIRSensor = readIRSensor(IR_MIDDLE);
//     d.rightIRSensor = readIRSensor(IR_RIGHT);
//     d.hallSensor = readHallSensor();
//     d.currentTime = millis();
//     d.clawSensor = readClawSensor(1);
//     d.IRSensor = 0;
//     return d;
// } 