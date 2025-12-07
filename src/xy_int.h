#ifndef _XY_INT_H_
#define _XY_INT_H_

extern float target_x_Pos; // target in mm
extern float target_y_Pos; // target in mm
extern volatile bool movement_x_done;
extern volatile bool movement_y_done;
void startup_XY();
void homingTask_x(void *pvParameters);
void homingTask_y(void *pvParameters);
void motionTask_x(void *pvParameters);
void motionTask_y(void *pvParameters);

#endif
