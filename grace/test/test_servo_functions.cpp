#include <unity.h>
#include <Arduino.h>

// Helper functions
long _map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
} 
  
int GET_DUTY_MG996R(int angle_val) {
    return ((int)_map( 
        _map((angle_val), 0, 180, 500, 2500), 
        0, (1000000L / 50), 0, ((1 << 12) - 1) 
    )); 
}

int GET_DUTY_25KGCM(int angle_val) {
    return ((int)_map( 
        _map((angle_val), 0, 180, 750, 2250), 
        0, (1000000L / 50), 0, ((1 << 12) - 1) 
    )); 
}

int angleToDutyMG996R(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 500, 2500);
    return (int)(pulseWidth * 4095) / 20000;
}

int angleToDuty25kgcm(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 750, 2250);
    return (int)(pulseWidth * 4095) / 20000;
}

// Test cases
void test_servo_mg996r_angles() {
    TEST_ASSERT_INT_WITHIN(5, GET_DUTY_MG996R(0), angleToDutyMG996R(0));
    TEST_ASSERT_INT_WITHIN(5, GET_DUTY_MG996R(90), angleToDutyMG996R(90));
    TEST_ASSERT_INT_WITHIN(5, GET_DUTY_MG996R(180), angleToDutyMG996R(180));
}

void test_servo_25kgcm_angles() {
    TEST_ASSERT_INT_WITHIN(5, GET_DUTY_25KGCM(0), angleToDuty25kgcm(0));
    TEST_ASSERT_INT_WITHIN(5, GET_DUTY_25KGCM(90), angleToDuty25kgcm(90));
    TEST_ASSERT_INT_WITHIN(5, GET_DUTY_25KGCM(180), angleToDuty25kgcm(180));
}

void test_mg996r_bounds() {
    // Test that angles are properly constrained
    TEST_ASSERT_EQUAL(angleToDutyMG996R(-10), angleToDutyMG996R(0));
    TEST_ASSERT_EQUAL(angleToDutyMG996R(200), angleToDutyMG996R(180));
}

void test_25kgcm_bounds() {
    // Test that angles are properly constrained
    TEST_ASSERT_EQUAL(angleToDuty25kgcm(-10), angleToDuty25kgcm(0));
    TEST_ASSERT_EQUAL(angleToDuty25kgcm(200), angleToDuty25kgcm(180));
}

void setup() {
    delay(2000);  // Give the serial monitor time to start
    UNITY_BEGIN();
    
    RUN_TEST(test_servo_mg996r_angles);
    RUN_TEST(test_servo_25kgcm_angles);
    RUN_TEST(test_mg996r_bounds);
    RUN_TEST(test_25kgcm_bounds);
    
    UNITY_END();
}

void loop() {
} 