#include <Arduino.h>
#include "xy_int.h"
#include "homing_flags.h"
#include "z_int.h"

#define grab_pin 10

void grabPart() { digitalWrite(grab_pin, HIGH); }
void releasePart() { digitalWrite(grab_pin, LOW); }

bool moveXYZ(float x_pos, float y_pos, float z_pos)
{
  // check if previous movement is done before modefying target coordinates
  if (movement_x_done && movement_y_done && movement_z_done)
  {
    target_x_Pos = x_pos;
    target_y_Pos = y_pos;
    target_z_Pos = z_pos;
    movement_x_done = false;
    movement_y_done = false;
    movement_z_done = false;
    return true;
  }
  else
  {
    return false;
  }
}

void movePart(void *parameter)
{
  while (1)
  { 
     
    while (moveXYZ(380, 195, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));

    while (moveXYZ(35, 65, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));

     while (moveXYZ(35, 125, 0) != true)
     {
       vTaskDelay(pdMS_TO_TICKS(1000));
     }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
     while (moveXYZ(35, 223, 0) != true)
     {
       vTaskDelay(pdMS_TO_TICKS(1000));
     }
      vTaskDelay(pdMS_TO_TICKS(1000));

    
     while (moveXYZ(35, 286, 0) != true)
     {
      vTaskDelay(pdMS_TO_TICKS(1000));
     }
      vTaskDelay(pdMS_TO_TICKS(1000));

    
     while (moveXYZ(90, 286, 0) != true)
     {
      vTaskDelay(pdMS_TO_TICKS(1000));
     }

      vTaskDelay(pdMS_TO_TICKS(1000));

    
     while (moveXYZ(90, 223, 0) != true)
     {
      vTaskDelay(pdMS_TO_TICKS(1000));
     }

      vTaskDelay(pdMS_TO_TICKS(1000));

    
     while (moveXYZ(90, 125, 0) != true)
     {
      vTaskDelay(pdMS_TO_TICKS(1000));
     }

      vTaskDelay(pdMS_TO_TICKS(1000));

    
     while (moveXYZ(90, 65, 0) != true)
     {
      vTaskDelay(pdMS_TO_TICKS(1000));
     }

      vTaskDelay(pdMS_TO_TICKS(1000));

   while (moveXYZ(144, 65, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
   while (moveXYZ(144, 125, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
   while (moveXYZ(144, 223, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
   while (moveXYZ(144, 286, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
      
    while (moveXYZ(199, 286, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (moveXYZ(199, 223, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (moveXYZ(199, 125, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (moveXYZ(199, 65, 0) != true)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
      vTaskDelay(pdMS_TO_TICKS(1000));

    
      /*
     grabPart();

     while (moveXYZ(100, 100, 0) != true)
     {
      vTaskDelay(pdMS_TO_TICKS(1000));
     }
    
     while (moveXYZ(100, 100, 50) != true)
     {
      vTaskDelay(pdMS_TO_TICKS(1000));
     }
     releasePart();
  }*/
}}

void setup()
{
  pinMode(grab_pin, OUTPUT);
  startup_XY();
  startup_Z();
  // Start homing tasks
  xTaskCreate(homingTask_x, "Homing_X", 4096, NULL, 3, NULL);     // create X homing task
  xTaskCreate(homingTask_y, "Homing_Y", 4096, NULL, 3, NULL);     // create Y homing task
  xTaskCreate(home_z, "home_z", 4096, NULL, 1, NULL);             // create Z homing task
  xTaskCreate(applyPID, "applyPID", 4096, NULL, 1, NULL);         // create Z motion task
  xTaskCreate(movePart, "movePart", 4096, NULL, 2, NULL);         // create part moving task in XY plane
  xTaskCreate(motionTask_x, "motionTask_x", 4096, NULL, 2, NULL); // create X motion task
  xTaskCreate(motionTask_y, "motionTask_y", 4096, NULL, 2, NULL); // create Y motion task
}

void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}
