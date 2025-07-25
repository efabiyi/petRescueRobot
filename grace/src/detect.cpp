// #include <Adafruit_VL53L0X.h>

// constexpr int SERVO_CHANNEL = 5;
// constexpr int SERVO_PIN = 0;

// struct PolarPoint {
//     int distance;
//     int angle;
// };

// struct CartesianPoint {
//     int x;
//     int y;
// };


// int angleToDutyMG996R(int angle) {
//     angle = constrain(angle, 0, 180);
//     int pulseWidth = map(angle, 0, 180, 500, 2500);
//     return (int) (pulseWidth * 4095) / 20000;
// }

// class Scanner {
// private:
//     static constexpr int SCAN_DATA_SIZE = 13;
//     static constexpr int SERVO_STEP_SIZE = 10;
//     static constexpr int MAX_ANGLE = 150;
//     static constexpr int MIN_ANGLE = 30;
//     static constexpr int MAX_DISTANCE = 9999;
//     static constexpr int IN_RANGE_DISTANCE = 300;
    
//     Adafruit_VL53L0X lox;
//     PolarPoint scanData[SCAN_DATA_SIZE];
//     int servoStep = SERVO_STEP_SIZE;
//     int servoAngle = 90;
    
// public:  
//     int readDistance();
//     void clearScanData();
//     void setServoAngle(int angle);

//     Scanner();
//     bool initialize();
//     void reset();
//     int getServoAngle() { return servoAngle; }
//     PolarPoint getClosestObject();
//     void scanOneStep();
//     bool completedScan();
//     void printScanData();
//     bool closestObjectIsWall();
//     PolarPoint honeIn(int angle);
// };


// Scanner::Scanner() {
//     servoStep = SERVO_STEP_SIZE;
//     servoAngle = 90;
// }

// bool Scanner::initialize() {
//     ledcSetup(SERVO_CHANNEL, 50, 12);
//     ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
//     Serial.println("Initializing VL53L0X sensor");
//     if (!lox.begin()) {
//         Serial.println(F("Failed to boot VL53L0X"));
//         return false;
//     }
//     reset();
//     return true;
// }

// void Scanner::reset() {
//     setServoAngle(90);
//     clearScanData();
// }

// int Scanner::readDistance() {
//     VL53L0X_RangingMeasurementData_t measure;
//     lox.rangingTest(&measure, false);

//     if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//         return measure.RangeMilliMeter + 15; // the laser face is 15mm in front of the center of the servo
//     } else {
//         return MAX_DISTANCE;
//     }
// }

// bool Scanner::completedScan() {
//     return servoAngle >= MAX_ANGLE || servoAngle <= MIN_ANGLE;
// }

// void Scanner::scanOneStep() {
//     if (completedScan()) {
//         servoStep = -1 * servoStep;
//     }
//     servoAngle = servoAngle + servoStep;
//     setServoAngle(servoAngle);
//     delay(100);
//     int distance = readDistance();
//     // Serial.println(distance);
//     int index = (servoAngle - MIN_ANGLE) / SERVO_STEP_SIZE;
//     scanData[index].distance = distance;
//     scanData[index].angle = servoAngle;
// }

// PolarPoint Scanner::honeIn(int angle) {
//     int minAngle = max(angle - 20, MIN_ANGLE);
//     int maxAngle = min(angle + 20, MAX_ANGLE);
//     int ang1, ang2;
//     int d1 = MAX_DISTANCE;
//     int d2 = MAX_DISTANCE;
//     PolarPoint obj;
//     obj.angle = -1;
//     obj.distance = MAX_DISTANCE;
//     for (int theta = minAngle; theta <= maxAngle; theta += 5) {
//         setServoAngle(theta);
//         delay(200);
//         int distance = readDistance();
//         if (distance < d1) {
//             ang1 = theta;
//             d1 = distance;
//         }
//     }
//     for (int theta = maxAngle; theta >= minAngle; theta -= 5) {
//         setServoAngle(theta);
//         delay(200);
//         int distance = readDistance();
//         if (distance < d2) {
//             ang2 = theta;
//             d2 = distance;
//         }
//     }
//     obj.angle = (ang1 + ang2) / 2;
//     obj.distance = (d1 + d2) / 2;
//     return obj;
// }

// void Scanner::setServoAngle(int angle) {
//     ledcWrite(SERVO_CHANNEL, angleToDutyMG996R(angle));
// }

// PolarPoint Scanner::getClosestObject() {
//     PolarPoint closestObject;
//     closestObject.distance = MAX_DISTANCE;
//     closestObject.angle = 0;

//     for (int i = 0; i < SCAN_DATA_SIZE; i++) {
//         if (scanData[i].distance < closestObject.distance) {
//             closestObject.distance = scanData[i].distance;
//             closestObject.angle = scanData[i].angle;
//         }
//     }

//     return closestObject;
// }

// void Scanner::clearScanData() {
//     for (int i = 0; i < SCAN_DATA_SIZE; i++) {
//         scanData[i].distance = MAX_DISTANCE;
//         scanData[i].angle = MIN_ANGLE + i * SERVO_STEP_SIZE;
//     }
// }

// void Scanner::printScanData() {
//     for (int i = 0; i < SCAN_DATA_SIZE; i++) {
//         Serial.println("Angle: " + String(scanData[i].angle) + " Distance: " + String(scanData[i].distance));
//     }
// }

// bool Scanner::closestObjectIsWall() {
//     int objectIndex = (getClosestObject().angle - MIN_ANGLE) / SERVO_STEP_SIZE;
//     int startIndex = max(objectIndex - 2, 0);
//     int endIndex = min(objectIndex + 2, SCAN_DATA_SIZE - 1);

//     CartesianPoint cartesianData[SCAN_DATA_SIZE];

//     for (int i = startIndex; i <= endIndex; i++) {
//         float radiansVal = radians(scanData[i].angle);
//         cartesianData[i].x = (scanData[i].distance) * cos(radiansVal);
//         cartesianData[i].y = (scanData[i].distance) * sin(radiansVal);
        
//         // Serial.print("angle: ");
//         // Serial.print(scanData[i].angle);
//         // Serial.print(", distance: ");
//         // Serial.print(scanData[i].distance);
//         // Serial.print(", x: ");
//         // Serial.print(cartesianData[i].x);
//         // Serial.print(", y: ");
//         // Serial.println(cartesianData[i].y);
//     }

//     float secondDeriv[SCAN_DATA_SIZE - 2];

//     for (int i = startIndex + 1; i <= endIndex - 1; i++) {
//         float prevY = cartesianData[i - 1].y;
//         float y = cartesianData[i].y;
//         float nextY = cartesianData[i + 1].y;

//         float h1 = cartesianData[i].x - cartesianData[i - 1].x;
//         float h2 = cartesianData[i + 1].x - cartesianData[i].x;
//         float denom1 = h1 * (h1 + h2);
//         float denom2 = h1 * h2;
//         float denom3 = h2 * (h1 + h2);

//         float deriv = 0.0;

//         if (denom1 != 0 && denom2 != 0 && denom3 != 0) {
//             deriv = 2.0f / denom1 * prevY - 2.0f / denom2 * y + 2.0f / denom3 * nextY;
//         } else {
//             deriv = 0.0;
//         }

//         secondDeriv[i] = deriv;

//         // Serial.print("deriv: ");
//         // Serial.println(deriv);
//         if (abs(deriv) > 0.02) {
//             return false;
//         }
//     }

//     return true;
// }
