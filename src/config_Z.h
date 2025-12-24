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

#endif
