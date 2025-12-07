#ifndef _Z_INT_H_
#define _Z_INT_H_

extern float target_z_Pos; // target in mm
extern volatile bool movement_z_done;

void startup_Z();
void home_z(void *pvParameters);
void applyPID(void *parameter);

#endif
