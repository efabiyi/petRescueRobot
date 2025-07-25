// #include <iostream>
// #include <random>
// #include <ctime>
// #include <array>
// #include <algorithm>
// #include <cmath>
// #include <Arduino.h>

// #define PI 3.14159265358979323846

// // Constants matching drive.cpp
// // const int BASE_SPEED = 780;
// // const int MIN_SPEED = 0;
// // const int MAX_SPEED = 3000;
// // const int THRESHOLD = 500;
// // const float KP = 5.0;
// // const float KD = 0.0;

// struct PolarPoint {
//     int distance;
//     int angle;
// };

// struct CartesianPoint {
//     int x;
//     int y;
// };

// // // Added from drive.cpp
// // const int OFF_LINE_THRESHOLD = 100;

// // std::array<int, 3> generateValidReadings() {
// //     static std::mt19937 rng(std::time(nullptr));
// //     // Updated range to match analog readings (0-4095 for ESP32)
// //     std::uniform_int_distribution<int> dist(0, 4095);
    
// //     int left, middle, right;
// //     int state = dist(rng) % 5; // Added a new state for off-line condition
    
// //     switch(state) {
// //         case 0: { // M > L > R
// //             left = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
// //             middle = dist(rng) % (4095 - left) + left;
// //             right = dist(rng) % left;
// //             break;
// //         }
// //         case 1: { // M > R > L
// //             right = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
// //             middle = dist(rng) % (4095 - right) + right;
// //             left = dist(rng) % right;
// //             break;
// //         }
// //         case 2: { // R > M > L
// //             right = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
// //             left = dist(rng) % THRESHOLD;
// //             middle = left + (right - left) / 2;
// //             break;
// //         }
// //         case 3: { // L > M > R
// //             left = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
// //             right = dist(rng) % THRESHOLD;
// //             middle = right + (left - right) / 2;
// //             break;
// //         }
// //         case 4: { // All sensors below threshold (off-line)
// //             left = dist(rng) % THRESHOLD;
// //             middle = dist(rng) % THRESHOLD;
// //             right = dist(rng) % THRESHOLD;
// //             break;
// //         }
// //     }
    
// //     return {left, middle, right};
// // }

// // void printState(const std::array<int, 3>& readings) {
// //     std::cout << "Left: " << readings[0] 
// //               << ", Middle: " << readings[1] 
// //               << ", Right: " << readings[2] << std::endl;
// // }

// // float calculateError(const std::array<int, 3>& readings) {
// //     float leftReading = readings[0];
// //     float middleReading = readings[1];
// //     float rightReading = readings[2];

// //     float error = (leftReading * -1.0 + rightReading * 1.0) / 
// //                 (leftReading + middleReading + rightReading + 0.001);

// //     return error;
// // }

// // bool isOffLine(const std::array<int, 3>& readings) {
// //     return (readings[0] < THRESHOLD && readings[1] < THRESHOLD && readings[2] < THRESHOLD);
// // }

// void computeDerivative(int* input, int* output, int size) {
//   for (int i = 0; i < size - 1; ++i) {
//     output[i] = input[i + 1] - input[i];
//     std::cout << output[i] << " ";
//   }
// }

// void findClusters(int* deriv, int size, int threshold, int minWidth) {
//   int start = -1;
//   int length = 0;

//   for (int i = 0; i < size; ++i) {
//     if (deriv[i] <= threshold) {
//       if (start == -1) start = i;
//       length++;
//     } else {
//       if (length >= minWidth - 1) {
//         std::cout << "Cluster found from index " << start << " to " << start + length << std::endl;
//       }
//       start = -1;
//       length = 0;
//     }
//   }

//   if (length >= minWidth - 1) {
//     std::cout << "Cluster found from index " << start << " to " << start + length << std::endl;
//   }
//   std::cout << std::endl;
// }


// void polarToCartesian(std::array<PolarPoint, 13>& polarData, std::array<CartesianPoint, 13>& cartesianData) {
//   for (int i = 0; i < 13; i++) {
//       cartesianData[i].x = polarData[i].distance * cos(polarData[i].angle * PI / 180);
//       cartesianData[i].y = polarData[i].distance * sin(polarData[i].angle * PI / 180);
//   }
// }


// #include <iostream>

// long _map(long x, long in_min, long in_max, long out_min, long out_max) {
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
// } 
  
// int GET_DUTY_MG996R(int angle_val) {
//   return ( (int)_map( 
//       _map( (angle_val) , 0, 180, 500, 2500), 
//       0, (1000000L / 50), 0, ((1 << 12) - 1) 
//     ) 
//   );
// }

// int GET_DUTY_25KGCM(int angle_val) {
//   return ( (int)_map( 
//       _map( (angle_val) , 0, 180, 750, 2250), 
//       0, (1000000L / 50), 0, ((1 << 12) - 1) 
//     ) 
//   );
// }

// int angleToDutyMG996R(int angle) {
//     angle = constrain(angle, 0, 180);
//     int pulseWidth = map(angle, 0, 180, 500, 2500);
//     return (int) (pulseWidth * 4095) / 20000;
// }

// int angleToDuty25kgcm(int angle) {
//     angle = constrain(angle, 0, 180);
//     int pulseWidth = map(angle, 0, 180, 750, 2250);
//     return (int) (pulseWidth * 4095) / 20000;
// }

// void testDutyFunctions() {
//     for (int angle = 0; angle <= 180; angle += 10) {
//         int duty1 = GET_DUTY_MG996R(angle);
//         int duty2 = angleToDutyMG996R(angle);
//         std::cout << "Angle: " << angle << " GET_DUTY_MG996R: " << duty1 << " angleToDutyMG996R: " << duty2 << std::endl;

//         int duty3 = GET_DUTY_25KGCM(angle);
//         int duty4 = angleToDuty25kgcm(angle);
//         std::cout << "Angle: " << angle << " GET_DUTY_25KGCM: " << duty3 << " angleToDuty25kgcm: " << duty4 << std::endl;
//     }
// }

// int main() {
//     testDutyFunctions();
//     return 0;
// }