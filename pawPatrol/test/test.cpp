#include <iostream>
#include <random>
#include <ctime>
#include <array>
#include <algorithm>
#include <cmath>

// Constants matching drive.cpp
const int BASE_SPEED = 780;
const int MIN_SPEED = 0;
const int MAX_SPEED = 3000;
const int THRESHOLD = 500;
const float KP = 5.0;
const float KD = 0.0;

// Added from drive.cpp
const int OFF_LINE_THRESHOLD = 100;

std::array<int, 3> generateValidReadings() {
    static std::mt19937 rng(std::time(nullptr));
    // Updated range to match analog readings (0-4095 for ESP32)
    std::uniform_int_distribution<int> dist(0, 4095);
    
    int left, middle, right;
    int state = dist(rng) % 5; // Added a new state for off-line condition
    
    switch(state) {
        case 0: { // M > L > R
            left = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
            middle = dist(rng) % (4095 - left) + left;
            right = dist(rng) % left;
            break;
        }
        case 1: { // M > R > L
            right = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
            middle = dist(rng) % (4095 - right) + right;
            left = dist(rng) % right;
            break;
        }
        case 2: { // R > M > L
            right = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
            left = dist(rng) % THRESHOLD;
            middle = left + (right - left) / 2;
            break;
        }
        case 3: { // L > M > R
            left = dist(rng) % (4095 - THRESHOLD) + THRESHOLD;
            right = dist(rng) % THRESHOLD;
            middle = right + (left - right) / 2;
            break;
        }
        case 4: { // All sensors below threshold (off-line)
            left = dist(rng) % THRESHOLD;
            middle = dist(rng) % THRESHOLD;
            right = dist(rng) % THRESHOLD;
            break;
        }
    }
    
    return {left, middle, right};
}

void printState(const std::array<int, 3>& readings) {
    std::cout << "Left: " << readings[0] 
              << ", Middle: " << readings[1] 
              << ", Right: " << readings[2] << std::endl;
}

float calculateError(const std::array<int, 3>& readings) {
    float leftReading = readings[0];
    float middleReading = readings[1];
    float rightReading = readings[2];

    float error = (leftReading * -1.0 + rightReading * 1.0) / 
                (leftReading + middleReading + rightReading + 0.001);

    return error;
}

bool isOffLine(const std::array<int, 3>& readings) {
    return (readings[0] < THRESHOLD && readings[1] < THRESHOLD && readings[2] < THRESHOLD);
}

int main() {
    float lastError = 0;
    float lastCorrection = 0;
    unsigned long lastTime = 0;
    int offLineCounter = OFF_LINE_THRESHOLD;

    for (int i = 0; i < 20; i++) {
        unsigned long now = i * 10; // Simulating 10ms intervals
        unsigned long deltaTime = now - lastTime;
        lastTime = now;

        auto readings = generateValidReadings();
        bool offLine = isOffLine(readings);

        printState(readings);

        if (offLine) {
            offLineCounter++;
            std::cout << "Off line detected! Counter: " << offLineCounter << std::endl;
            
            if (offLineCounter >= OFF_LINE_THRESHOLD) {
                std::cout << "Stopped - Off line for too long" << std::endl;
                std::cout << "Left Speed: 0" << std::endl;
                std::cout << "Right Speed: 0" << std::endl;
            } else {
                // Continue in last known direction
                if (lastCorrection > 0) {
                    std::cout << "Off line recovery - turning right" << std::endl;
                    std::cout << "Left Speed: 0" << std::endl;
                    std::cout << "Right Speed: " << MAX_SPEED << std::endl;
                } else {
                    std::cout << "Off line recovery - turning left" << std::endl;
                    std::cout << "Left Speed: " << MAX_SPEED << std::endl;
                    std::cout << "Right Speed: 0" << std::endl;
                }
            }
        } else {
            offLineCounter = 0;
            float error = calculateError(readings);
            float derivative = (error - lastError) / (deltaTime / 1.0);
            float correction = (KP * error) + (KD * derivative);
            
            int leftSpeed = BASE_SPEED - (correction * BASE_SPEED/2);
            int rightSpeed = BASE_SPEED + (correction * BASE_SPEED/2);

            leftSpeed = std::max(leftSpeed, MIN_SPEED);
            leftSpeed = std::min(leftSpeed, MAX_SPEED);
            rightSpeed = std::max(rightSpeed, MIN_SPEED);
            rightSpeed = std::min(rightSpeed, MAX_SPEED);

            std::cout << "Error: " << error << std::endl;
            std::cout << "Derivative: " << derivative << std::endl;
            std::cout << "Correction: " << correction << std::endl;
            std::cout << "Left Speed: " << leftSpeed << std::endl;
            std::cout << "Right Speed: " << rightSpeed << std::endl;

            lastError = error;
            lastCorrection = correction;
        }
        
        std::cout << "--------------------------------" << std::endl;
        std::cout << std::endl;
    }
    
    return 0;
}