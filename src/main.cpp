#include <Arduino.h>
#include <string.h>
#include "xy_int.h"
#include "homing_flags.h"
#include "z_int.h"
#include "Sender.h"
#include "Reciver.h"
#include "positions.h"


#define grab_pin 10
#define start_pin 36
#define stop_pin 39

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

class sortPart
{
  public:
    bool isOccupied=false;
    bool isWhite=false;
    bool isBase=false;
};

class reservePart : public sortPart
{
};


class assemblyPart : public sortPart
{
  public:
    bool hasLid=false;
};



sortPart currentPart;
assemblyPart assemblyParts[8];
reservePart reserveParts[8];



void movePart(int from_x, int from_y, int from_z, int to_x, int to_y, int to_z)
{ 
  while (moveXYZ(from_x, from_y, 0) != true)
  {
    delay(1000);
  }
  delay(1000);
  while (moveXYZ(from_x, from_y, from_z) != true)
  {
    delay(1000);
  }
  delay(1000);
  grabPart();
  delay(1000);

  while (moveXYZ(from_x, from_y, 0) != true)
  {
    delay(1000);
  }
  delay(1000);

  while (moveXYZ(to_x, to_y, 0) != true)
  {
    delay(1000);
  }
  delay(1000);

  while (moveXYZ(to_x, to_y, to_z) != true)
  {
    delay(1000);
  }
  delay(1000);
  releasePart();
  delay(1000);
}

void partAssembly(void *parameter)
{
  while (!homingDone_x || !homingDone_y || !homingDone_z) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Send("S");
    delay(1000);
  while (1)
  {
    currentPart.isOccupied=false;

    char msg = Receive()[0];
    switch (msg)
    {
    case 'A':
      currentPart.isOccupied=true;
      currentPart.isBase=true;
      currentPart.isWhite=true;
      break;
    case 'B':
      currentPart.isOccupied=true;
      currentPart.isBase=false;
      currentPart.isWhite=true;
      break;
    case 'C':
      currentPart.isOccupied=true;
      currentPart.isBase=true;
      currentPart.isWhite=false;
      break;
    case 'D':
      currentPart.isOccupied=true;
      currentPart.isBase=false;
      currentPart.isWhite=false;
      break;
    case 'N':
      currentPart.isOccupied=false;
    delay(1000);
      break;
    default:
    delay(1000);
      break;
    }

    //managing base parts
    if (currentPart.isOccupied && currentPart.isBase)
    {
      int i=0;
      int j=0;
      for (i;i<8;i++)
      {
        if(!(assemblyParts[i].isOccupied))
        {
          break;
        }
      }
      movePart(sortBox.x,sortBox.y,z_base,assemBox[i].x,assemBox[i].y,z_base);
      assemblyParts[i].isOccupied=true;
      assemblyParts[i].isBase=true;
      assemblyParts[i].isWhite=currentPart.isWhite;
      assemblyParts[i].hasLid=false;
      for(j;j<8;j++)
      {
        //if a lid of the same color exists
        if(reserveParts[j].isOccupied && (reserveParts[j].isWhite==currentPart.isWhite))
        {
          movePart(reserveBox[j].x,reserveBox[j].y,z_lid,assemBox[i].x,assemBox[i].y,z_lid_on_base);
          reserveParts[j].isOccupied=false;
          assemblyParts[i].hasLid=true;
          break;
        }
      }
      Send("F");
      
    }
    //managing lid parts
    else if(currentPart.isOccupied && (!currentPart.isBase))
    {
      int i=0;
      int j=0;
      for(i;i<8;i++)
      {
        //if a base of same color already exists
        if((!assemblyParts[i].hasLid) && assemblyParts[i].isOccupied && (assemblyParts[i].isWhite==currentPart.isWhite))
        {
          movePart(sortBox.x,sortBox.y,z_lid,assemBox[i].x,assemBox[i].y,z_lid_on_base);
          currentPart.isOccupied=false;
          assemblyParts[i].hasLid=true;
          break;
        }
      }
      if(currentPart.isOccupied)
      {
        for(j; j<8;j++)
        {
          if(!(reserveParts[j].isOccupied))
          {
            break;
          }
        }
        movePart(sortBox.x,sortBox.y,z_lid,reserveBox[j].x,reserveBox[j].y,z_lid);
        reserveParts[j].isOccupied=true;
        reserveParts[j].isWhite=currentPart.isWhite;
        reserveParts[j].isBase=false;
      }
      Send("F");
    }

    delay(1000);
  }
}

void setup()
{
  pinMode(grab_pin, OUTPUT);
  pinMode(start_pin,INPUT_PULLUP);
  pinMode(stop_pin,INPUT_PULLUP);
  startup_XY();
  startup_Z();
  Sender_Init();
  Receiver_Init();
  // Start homing tasks
  xTaskCreate(homingTask_x, "Homing_X", 4096, NULL, 3, NULL);     // create X homing task
  xTaskCreate(homingTask_y, "Homing_Y", 4096, NULL, 3, NULL);     // create Y homing task
  xTaskCreate(home_z, "home_z", 4096, NULL, 3, NULL);             // create Z homing task
  xTaskCreate(applyPID, "applyPID", 4096, NULL, 2, NULL);         // create Z motion task
  xTaskCreate(partAssembly, "partAssembly", 4096, NULL, 1, NULL); // create part moving task in XY plane
  xTaskCreate(motionTask_x, "motionTask_x", 4096, NULL, 2, NULL); // create X motion task
  xTaskCreate(motionTask_y, "motionTask_y", 4096, NULL, 2, NULL); // create Y motion task
}

void loop()
{
}
