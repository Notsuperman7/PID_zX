#include <Arduino.h>
#include "xy_int.h"
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
    while (moveXYZ(0, 0, 50) != true)
    {
      delay(100);
    }
    grabPart();

    while (moveXYZ(100, 100, 0) != true)
    {
      delay(100);
    }
    while (moveXYZ(100, 100, 50) != true)
    {
      delay(100);
    }
    releasePart();
  }
}

void setup()
{
  pinMode(grab_pin, OUTPUT);
  startup_XY();
  startup_Z();
  xTaskCreate(movePart, "movePart", 4096, NULL, 1, NULL);
}

void loop()
{
}