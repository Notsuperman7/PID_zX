#ifndef _CONFIG_H_
#define _CONFIG_H_

// Define X/Y axis pin connections
#define dirPin_x 23
#define stepPin_x 22
#define limitSwitchPin_x 21
#define limitSwitchPin_y 19
#define dirPin_y 18
#define stepPin_y 5

// Define X/Y axis constants
// X Axis Motor Configuration
constexpr int maxTravelDistance_x = 380; // 430mm
constexpr int stepsPerMM_x = 25;         // steps per mm for the motor  => pitch = 8mm, 200 steps/rev, 16 microsteps
constexpr int microsteps_x = 16;         // microstepping setting
// Y Axis Motor Configuration
constexpr int maxTravelDistance_y = 420; // 420mm (corrected from 430mm in comment)
constexpr float stepsPerMM_y = 10.75;         // steps per mm for the motor
constexpr int microsteps_y = 16;         // microstepping setting

#endif
