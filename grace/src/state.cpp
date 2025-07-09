// #include "state.h"
// #include "sensors.h"
// #include "detect.h"

// const int HALL_THRESHOLD = 2000;

// State getState() {
//     if (currentState != DRIVE) {
//         return currentState;
//     }
    
//     if (currentData.leftIRSensor < 500 && currentData.middleIRSensor < 500 && currentData.rightIRSensor < 500) {
//         return ESCAPE;
//     }
    
//     if (scanForPets()) {
//         ScanResult target = getLastScanResult();
        
//         if (target.distance < 300) {
//             if (alignWithMagnet()) {
//                 return GRAB_PET;
//             }
//         }
//     }
    
//     return DRIVE;
// } 