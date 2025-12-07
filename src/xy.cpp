// Include the FastAccelStepper Library
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config_XY.h"
#include "homing_flags.h"
// Creates an instance
FastAccelStepperEngine engine;
FastAccelStepper *Stepper_x = nullptr;
FastAccelStepper *Stepper_y = nullptr;

float target_x_Pos = 00.0;
float target_y_Pos = 00.0;

volatile bool movement_x_done = true;
volatile bool movement_y_done = true;

volatile bool homingDone_x = false;
volatile bool homingDone_y = false;

// Function prototypes
void homingTask_x(void *pvParameters);
void homingTask_y(void *pvParameters);
// void homingTask(void *pvParameters);
void motionTask_x(void *pvParameters);
void motionTask_y(void *pvParameters);

void startup_XY()
{
    Serial.begin(115200); // Added for debugging
    Serial.println("Starting setup...");

    // Configure limit switches
    pinMode(limitSwitchPin_x, INPUT_PULLUP); // pull-up resistor
    pinMode(limitSwitchPin_y, INPUT_PULLUP); // pull-up resistor

    // Initialize stepper engine and connect steppers
    engine.init();
    Stepper_x = engine.stepperConnectToPin(stepPin_x);
    Stepper_y = engine.stepperConnectToPin(stepPin_y);

    // Check if stepper connections were successful
    if (!Stepper_x || !Stepper_y)
    {
        Serial.println("Failed to connect steppers!");
        return;
    }

    // Configure X and Y axis stepper motors
    if (Stepper_x)
    {
        Stepper_x->setDirectionPin(dirPin_x);
        Stepper_x->setAutoEnable(true);
        Stepper_x->setAcceleration(1500); // Set acceleration
        Stepper_x->setSpeedInHz(2000);    // Set max speed
    }
    if (Stepper_y)
    {
        Stepper_y->setDirectionPin(dirPin_y);
        Stepper_y->setAutoEnable(true);
        Stepper_y->setAcceleration(1500); // Set acceleration
        Stepper_y->setSpeedInHz(2000);    // Set max speed
    }

    Serial.println("Steppers configured, starting homing...");
    
    xTaskCreate(homingTask_x, "Homing_X", 4096, NULL, 2, NULL);
    xTaskCreate(homingTask_y, "Homing_Y", 4096, NULL, 2, NULL);
    // Start motion tasks
    xTaskCreate(motionTask_x, "Motion X", 4096, NULL, 1, NULL);
    xTaskCreate(motionTask_y, "Motion Y", 4096, NULL, 1, NULL);
}

void motionTask_x(void *pvParameters)
{
    // Wait for homing to complete
    while (!homingDone_x || !homingDone_y || !homingDone_z)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("Motion X task proceeding after homing");

    int targetSteps_x = (int)(constrain(target_x_Pos, 0, maxTravelDistance_x) * stepsPerMM_x * microsteps_x);
    Serial.print("Moving X to position: ");
    Serial.println(targetSteps_x);

    if (Stepper_x)
    {
        // Issue non-blocking move; FastAccelStepper handles stepping via timer
        Stepper_x->moveTo(targetSteps_x);
    }

    // Keep task alive and monitor movement
    while (1)
    {
        if (Stepper_x && !Stepper_x->isRunning())
        {
            Serial.println("X movement completed");
            movement_x_done = true;
            break; // Exit if movement is done
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Keep task alive
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void motionTask_y(void *pvParameters)
{
    // Wait for homing to complete
    while (!homingDone_x || !homingDone_y || !homingDone_z)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("Motion Y task proceeding after homing");

    int targetSteps_y = (int)(constrain(target_y_Pos, 0, maxTravelDistance_y) * stepsPerMM_y * microsteps_y);
    Serial.print("Moving Y to position: ");
    Serial.println(targetSteps_y);

    if (Stepper_y)
    {
        // Issue non-blocking move; FastAccelStepper handles stepping via timer
        Stepper_y->moveTo(targetSteps_y);
    }

    // Keep task alive and monitor movement
    while (1)
    {
        if (Stepper_y && !Stepper_y->isRunning())
        {
            Serial.println("Y movement completed");
            movement_y_done = true;
            break; // Exit if movement is done
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Keep task alive
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void homingTask_x(void *pvParameters)
{
    if (!Stepper_x)
        vTaskDelete(NULL);

    Serial.println("Homing X axis...");

    // Move towards the limit switch at a safe homing speed
    Stepper_x->setSpeedInHz(2000);        // positive speed
    Stepper_x->setDirectionPin(dirPin_x); // Ensure direction pin is set
    // Set direction by moving in negative direction (backward)
    Stepper_x->move(-1000000); // Move a large negative distance until limit hit

    while (digitalRead(limitSwitchPin_x) == HIGH)
    { // assuming HIGH means not triggered
        // Let FastAccelStepper handle the movement through run() in loop()
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Stop and set position zero
    Stepper_x->stopMove();
    Stepper_x->setCurrentPosition(0);
    homingDone_x = true;
    Serial.println("X axis homed to position 0");

    vTaskDelete(NULL);
}

void homingTask_y(void *pvParameters)
{
    if (!Stepper_y)
        vTaskDelete(NULL);

    Serial.println("Homing Y axis...");

    // Move towards the limit switch at a safe homing speed
    Stepper_y->setSpeedInHz(2000);        // positive speed
    Stepper_y->setDirectionPin(dirPin_y); // Ensure direction pin is set
    // Set direction by moving in negative direction (backward)
    Stepper_y->move(-1000000); // Move a large negative distance until limit hit

    while (digitalRead(limitSwitchPin_y) == HIGH)
    { // assuming HIGH means not triggered
        // Let FastAccelStepper handle the movement through run() in loop()
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Stop and set position zero
    Stepper_y->stopMove();
    Stepper_y->setCurrentPosition(0);
    homingDone_y = true;
    Serial.println("Y axis homed to position 0");

    vTaskDelete(NULL);
}