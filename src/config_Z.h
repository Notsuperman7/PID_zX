#ifndef _CONFIG_H_
#define _CONFIG_H_

// Define Z axis pin connections
#define IN1 2
#define IN2 4
#define ENA 15
#define ENC_A 26
#define ENC_B 25
#define limitSwitchPin_z 14
    
// Define Z axis constants

// Z Axis Motor Configuration
constexpr int PPR = 374;        // pulses per revolution (as you defined)
constexpr float screw_lead = 8; // linear mm per revolution (screw pitch)
/*
// PID struct
struct PID
{
    float Kp, Ki, Kd;
    float integral, prevError;
    float outMin, outMax;
};

// Tune these later if needed
PID posPID = {10, 0.7, 5.3, 0.0, 0.0, -200.0, 200.0};
PID velPID = {1, 0.1, 0.0, 0.0, 0.0, -255.0, 255.0};
*/
#endif
