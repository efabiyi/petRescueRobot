// // #include "IR_remote.h"
// // #include "keymap.h"

// // IRremote ir(3);

// #include <Servo.h>
// #include "Adafruit_VL53L0X.h"

// Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// int loop_step_delay = 50; // Delay in milliseconds for each loop iteration
// int scanner_step = 10;
// int running_scanner_step = 10;  // 10 degrees per step
// int scanner_degrees_max = 130;   //end at 130 degrees 
// int scanner_degrees_min = 50;  //start at 50 degrees 
// int MAX_DISTANCE = 9999;
// int in_range_distance = 300; // Distance in mm to consider an item in range
// int reachable_distance = 95; // Distance in mm to consider an item reachable

// struct ScanDataItem {
//     int distance; 
//     int angle;
//     bool valid;
// };

// int scanner_degrees = 90; // Initial position of the scanner servo
// int base_degrees = 90; // Initial position of the base servo
// int arm_degrees = 30; // Initial position of the arm servo
// int claw_degrees = 90; // Initial position of the claw servo

// Servo servoScan;
// Servo servoArm;
// Servo servoBase;
// Servo servoClaw;

// int totalNumber = 15; 
// struct ScanDataItem scanData[15]; // Array to hold scan data

// // Debugging function to print messages to Serial
// void Debug_Print(const String &message) {
//   Serial.println(message);     //Comment this out to improve performance
// }

// void reset() {
//   // Debug_Print("Restarting...");
//   Clear_ScanData();
  
//   loop_step_delay = 100; // Delay in milliseconds for each loop iteration
//   scanner_degrees = 90;  //scanner_degrees_min;
//   base_degrees = 90;
//   arm_degrees = 30;
//   claw_degrees = 90;
//   servoScan.attach(12);
//   servoArm.attach(10);
//   servoBase.attach(11);
//   servoClaw.attach(9);

//   // SetServoScan(scanner_degrees);
//   // SetServoArm(arm_degrees);
//   // SetServoBase(base_degrees);
//   // SetServoClaw(claw_degrees);

//   servoScan.write(scanner_degrees);
//   servoArm.write(arm_degrees);
//   servoBase.write(base_degrees);
//   servoClaw.write(claw_degrees);

//   delay(500);
  
// }

// void setup()
// {
//   reset();
  
//   Stop();
//   pinMode(2, OUTPUT);
//   pinMode(5, OUTPUT);
//   pinMode(4, OUTPUT);
//   pinMode(6, OUTPUT);

//   pinMode(7, INPUT);//Left line tracking sensor is connected to the digital IO port D7
//   // pinMode(8, INPUT);//Center line patrol sensor is connected to the digital IO port D8
//   pinMode(8, INPUT);//Right line tracking sensor is connected to the digital IO port D9

//   Serial.begin(9600);

//   while (! Serial) {
//     delay(1);
//   }

//   Debug_Print("Adafruit VL53L0X test");
//   if (!lox.begin()) {
//     Debug_Print(F("Failed to boot VL53L0X"));
//     while(1);
//   }
// }

// void KeepMoving(int speed) {
//   ////// ---  Following the line --- Begin
//   int Left_Tra_Value = 1;
//   // int Center_Tra_Value = 1;
//   int Right_Tra_Value = 1;
//   Left_Tra_Value = digitalRead(7);
//   // Center_Tra_Value = digitalRead(8);
//   Right_Tra_Value = digitalRead(8);

//   if (Left_Tra_Value == 0 && Right_Tra_Value == 0) {
//     Rotate_Right(speed);
//     // delay(800);
//   } else if (Left_Tra_Value == 1 && Right_Tra_Value == 1) {
//     Rotate_Left(speed);
//     // delay(800);
//   } else {
//     Move_Forward(speed);
//   }
//   ////// ---  Following the line --- End

// }

// void loop(){
// // return;
//   KeepMoving(50); // Move forward at speed 500

//   // Check closest object after spining the scanner one round. (50-130  or 130-50)
//   if (scanner_degrees > scanner_degrees_max || scanner_degrees < scanner_degrees_min) {
//     running_scanner_step = (-1) * running_scanner_step;

//     ScanDataItem closestItem = GetClosestObject();

//     if (closestItem.valid == 1) {
//       if (closestItem.distance < in_range_distance) {
//         Stop();
//         Debug_Print("Found closestItem: " + String(closestItem.angle) + " distance: " + String(closestItem.distance));
        
//         base_degrees = base_degrees - (closestItem.angle - 90);
//         closestItem = Adjust_Bass_Servo(base_degrees, 5, 5); // Adjust the base servo to face the item
//         Debug_Print("Adjust_Bass_Servo: closestItem: " + String(closestItem.angle) + " distance: " + String(closestItem.distance));
 
//         while (closestItem.distance > reachable_distance) { // If the item is too far, move closer
//           Debug_Print("Moving closer to item at angle: " + String(closestItem.angle) + " distance: " + String(closestItem.distance));
//           KeepMoving(50); // Move forward at speed 50
//           delay(300); // Wait for a second before checking again
//           Stop(); // Stop the robot to read the distance again
//           closestItem = Adjust_Bass_Servo(base_degrees, 5, 5); // Get the closest item again

//           // Check if the closest item is still valid and within range
//           if (closestItem.valid == false || closestItem.distance > in_range_distance) {
//             Debug_Print("No valid item found, resetting...");
//             reset(); // Reset the robot if no valid item found
//             break; // Exit the loop
//           }

//         }

//         if (closestItem.distance < reachable_distance) {
//           Debug_Print("Item is reachable, picking up...");
//           PickUp(); // Call the function to pick up the item
//         }

//       }
//     } else {
//       reset(); // Reset the robot if no valid scan data found
//     }
//     Clear_ScanData();
//   } 

//   // Scan one step & read distance
//   scanner_degrees = scanner_degrees + running_scanner_step;
//   servoScan.write(scanner_degrees);
//   // Debug_Print("scanner_degrees : " + String(scanner_degrees));

//   int distance = Read_scanner();
//   int pos = (scanner_degrees - scanner_degrees_min) / scanner_step;
//   Save_ScanData(scanner_degrees, distance, pos);

//   //Wait for hardware complete their movement or actions
//   delay(loop_step_delay);

// }

// int Read_scanner() {
//   VL53L0X_RangingMeasurementData_t measure;
    
//   lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

//   if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//     // Debug_Print("Distance: " + String(measure.RangeMilliMeter) + " mm");
//     return measure.RangeMilliMeter;
//   } else {
//     return MAX_DISTANCE; // out of range
//   }
// }

// void PickUp() { //Not done yet
//   Stop();
//   int action_delay = 100; // Delay in milliseconds for each action

//   int claw_distance = 36; // distance in mm

//   //Adjust_Bass_Servo to face the item
//   ScanDataItem theItem = Adjust_Bass_Servo(base_degrees, 3, 3);

//   Debug_Print("Pick up: Adjust_Bass_Servo: " + String(theItem.angle) + " distance: " + String(theItem.distance));
  
//   SetServoArm(20); // Move the arm to the initial position
//   delay(action_delay);
//   int current_distance = MAX_DISTANCE;
//   // Move the arm to pick up the item
//   while (current_distance > claw_distance)  // Replace 'condition' with the actual condition to check if the item is reachable
//   {
//     int next_arm_degrees = arm_degrees + 5; // Move the arm up

//     Debug_Print("Move closer: next_arm_degrees: " + String(next_arm_degrees) + " current_distance: " + String(current_distance) + " claw_distance: " + String(claw_distance));
//     if (next_arm_degrees > 135 || next_arm_degrees < 20) { // Limit the arm movement to 20-140 degrees
//       SetServoArm(20); // Move the arm to the initial position
//       break;
//     }
//     SetServoArm(next_arm_degrees); // Move the arm down
//     delay(action_delay);
//     current_distance = Read_scanner();
//     if (current_distance < claw_distance) {
//       SetServoClaw(90); // Move the claw to close it
//       delay(action_delay);
//       while (claw_degrees < 170) // Close the claw
//       {
//         SetServoClaw(claw_degrees + 5); // Move the claw to close it
//         delay(action_delay);
//       }

//       delay(1000); // Wait for the claw to close

//       //put aside the item
//       while (base_degrees > 0) // Close the claw
//       {
//         if (arm_degrees > 40) {
//           SetServoArm(arm_degrees - 10); // Move the arm to the initial position
//         }
//         SetServoBase(base_degrees - 10); // Move the claw to close it
//         delay(action_delay);
//       }
//         SetServoClaw(90); // Move the claw to close it
//         delay(action_delay);
//       break; // Exit the loop if the item is picked up
//     }
//   }
  
//   // Debug_Print("servoBase: " + String(base_degrees) + " arm_degrees: " + String(arm_degrees) + " claw_degrees: " + String(claw_degrees));
//   // Debug_Print("PickUpAt: " + String(closestItem.angle) + " distance: " + String(closestItem.distance));


//   delay(2000); // Wait for the robot to stop moving
//   reset(); // Reset the robot to the initial state
// }

// ScanDataItem Adjust_Bass_Servo(int straight_scanner_degrees, int scan_step, int scan_positions_one_side) {
//   int action_delay = loop_step_delay * 2; // Delay in milliseconds for each action


//   int start_angle = straight_scanner_degrees - (scan_positions_one_side * scan_step);
//   int end_angle = straight_scanner_degrees + (scan_positions_one_side * scan_step);
//   Debug_Print("start_angle: " + String(start_angle) + " end_angle: " + String(end_angle));

//   //Straight scanner servo to 90 degrees
//   SetServoScan(90);
//   SetServoBase(start_angle);
//   delay(action_delay * 3); // Wait for the scanner to move

//   Clear_ScanData();
//   for(int angle = start_angle; angle <= end_angle; angle = angle + scan_step) {
//     SetServoBase(angle);
//     delay(action_delay); 
//     int distance = Read_scanner();
//     int pos = (angle - start_angle) / scan_step;
//     Save_ScanData(angle, distance, pos);
//   }

//   // Print_ScanData();
//   ScanDataItem closestItem = GetClosestObject();
//   SetServoBase(closestItem.angle); // Set the base servo to the angle of the closest item
//   delay(action_delay); 

//   return closestItem;
// }

// ScanDataItem GetClosestObject() { //
//   int distance_closest = 1000;
//   ScanDataItem closestItem;
//   closestItem.angle = 90;
//   closestItem.distance = MAX_DISTANCE;
//   closestItem.valid = false;

//   // Debug_Print("GetClosestObject");
//   for(int a = 0; a < totalNumber; a++)
//   {
//     ScanDataItem item = scanData[a];
//     // Debug_Print("item angle: " + String(item.angle) + " distance: " + String(item.distance) + " valid: " + String(item.valid));
//     if (item.valid > 0) {
//       if (distance_closest > item.distance && item.distance > 0 ) {
//         closestItem = item;
//         distance_closest = item.distance;
//         // Debug_Print("closestItem: " + String(closestItem.angle) + " distance: " + String(closestItem.distance));
//       }
//     }
//   }

//   return closestItem;
  
// }

// void Clear_ScanData() {
//   for(int a = 0; a < totalNumber; a++)
//   {
//     scanData[a].angle = 0;
//     scanData[a].distance = MAX_DISTANCE;
//     scanData[a].valid = false;
//   }
// }

// void Print_ScanData() {
//   Debug_Print("Print_ScanData");
//   for(int a = 0; a < totalNumber; a++)
//   {
//     ScanDataItem item = scanData[a];
//     Debug_Print("item angle: " + String(item.angle) + " distance: " + String(item.distance) + " valid: " + String(item.valid));
//   }
// }

// void Save_ScanData(int angle, int distance, int pos) {
//   scanData[pos].angle = angle;
//   scanData[pos].distance = distance;
//   scanData[pos].valid = true;
//   // Debug_Print("Saving scan data at angle: " + String(angle) + " distance: " + String(distance));
//   ScanDataItem item = scanData[pos];
//   // Debug_Print("save item at pos: " + String(pos) + "angle: " + String(item.angle) + " distance: " + String(item.distance) + " valid: " + String(item.valid));
// }

// void SetServoArm(int angle)
// {
//   // Debug_Print("SetServoArm: " + String(angle));
//   if (angle <= 180 && angle >= -10)
//   {
//     arm_degrees = angle;
//     servoArm.write(arm_degrees);
//   }
// }

// void SetServoClaw(int angle)
// {
//   // Debug_Print("SetServoClaw: " + String(angle));
//   if (angle <= 180 && angle >= -10)
//   {
//     claw_degrees = angle;
//     servoClaw.write(claw_degrees);
//   }
// }

// void SetServoScan(int angle)
// {
//   // Debug_Print("SetServoScan: " + String(angle));
//   if (angle <= 180 && angle >= -10)
//   {
//     scanner_degrees = angle;
//     servoScan.write(scanner_degrees);
//   }
// }

// void SetServoBase(int angle)
// {
//   Debug_Print("SetServoBase: " + String(angle));
//   Debug_Print("base_degrees: " + String(base_degrees));

//   // int step = 10;
//   // int isIncreasing = 1;
//   // if (angle >= base_degrees)
//   // {
//   //   isIncreasing = 1; // Increasing angle
//   // }
//   // else
//   // {
//   //   isIncreasing = -1; // Decreasing angle
//   // }
//   // int diff = isIncreasing * (angle - base_degrees); // Calculate the absolute difference between the angles
//   // Debug_Print("isIncreasing: " + String(isIncreasing));
//   // Debug_Print(" diff: " + String(diff) + " step: " + String(step));

//   // while (diff > step) { // Allow a margin of 10 degrees
//   //   base_degrees = base_degrees + (isIncreasing * step); // Adjust the base servo angle
//   //   servoBase.write(base_degrees);
//   //   delay(100); // Wait for the servo to move
//   //   Debug_Print("step SetServoBase: " + String(base_degrees));
//   //   diff = isIncreasing * (angle - base_degrees); // Calculate the absolute difference between the angles

//   // }

//   if (angle <= 180 && angle >= -10)
//   {
//     base_degrees = angle;
//     servoBase.write(base_degrees);
//   }
// }

// void Move_Backward(int speed) {
//   digitalWrite(2,LOW);
//   analogWrite(5,speed);
//   digitalWrite(4,HIGH);
//   analogWrite(6,speed);
// }

// void Rotate_Right(int speed) {
//   digitalWrite(2,HIGH);
//   analogWrite(5,speed);
//   digitalWrite(4,HIGH);
//   analogWrite(6,speed);
// }

// void Rotate_Left(int speed) {
//   digitalWrite(2,LOW);
//   analogWrite(5,speed);
//   digitalWrite(4,LOW);
//   analogWrite(6,speed);
// }

// void Stop() {
//   digitalWrite(2,LOW);
//   analogWrite(5,0);
//   digitalWrite(4,HIGH);
//   analogWrite(6,0);
//   delay(1000);
// }

// void Move_Forward(int speed) {
//   digitalWrite(2,HIGH);
//   analogWrite(5,speed);
//   digitalWrite(4,LOW);
//   analogWrite(6,speed);
// }
