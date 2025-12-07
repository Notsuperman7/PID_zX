#include <Arduino.h>
#include "config_Z.h"
#include "homing_flags.h"

portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

float target_z_Pos = 00.0; // target in mm
volatile bool movement_z_done = true;
volatile bool homingDone_z = false;

volatile long encoderCount = 0;

unsigned long lastTime = 0;
long lastCount = 0;

// ---------------- Encoder ISR ----------------
// single interrupt on ENC_A, sample ENC_B inside ISR
void IRAM_ATTR encoderISR()
{
    // read both lines quickly
    bool a = digitalRead(ENC_A);
    bool b = digitalRead(ENC_B);

    portENTER_CRITICAL_ISR(&encMux);
    // standard quadrature decoding (choose sign that matches your wiring)
    if (a == b)
        encoderCount--; // direction sign chosen to match your wiring earlier
    else
        encoderCount++;
    portEXIT_CRITICAL_ISR(&encMux);
}

void setupPWM()
{
    ledcSetup(0, 20000, 8); // channel 0, 20kHz, 8-bit
    ledcAttachPin(ENA, 0);  // attach ch 0 to ENA pin
    ledcWrite(0, 0);
}

void setMotor(int pwm)
{

    ledcWrite(0, abs(pwm));
    if (pwm > 0)
    {
        // DOWN
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else if (pwm < 0)
    {
        // UP
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
}
void home_z(void *pvParameters)
{
    Serial.println("Homing z axis...");
    setMotor(-200);
    vTaskDelay(pdMS_TO_TICKS(50)); // Let motor start
    while (digitalRead(limitSwitchPin_z) == HIGH)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    setMotor(0);
    portENTER_CRITICAL(&encMux);
    encoderCount = 0;
    portEXIT_CRITICAL(&encMux);

    homingDone_z = true;
    Serial.println("Z axis homed to position 0");

    vTaskDelete(NULL);
}

float computeDistanceMM(long count)
{
    float rev = (count / (float)PPR); // revolutions
    return rev * screw_lead;          // linear mm
}

// ---------------- PID compute ----------------
float computePID(PID &pid, float setpoint, float measurement, float deltaTime)
{
    float error = setpoint - measurement;
    pid.integral += 0.5f * (error + pid.prevError) * deltaTime; // trapezoidal integration
    pid.integral = constrain(pid.integral, pid.outMin, pid.outMax);
    float derivative = (error - pid.prevError) / deltaTime;
    float out = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.prevError = error;
    return constrain(out, pid.outMin, pid.outMax);
}

void applyPID(void *parameter)
{
    while (!homingDone_x || !homingDone_y || !homingDone_z)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println("Starting Z PID control loop...");

    lastTime = micros();
    lastCount = encoderCount;
    static float lastTarget = -999.0f;
    unsigned long lastDebugTime = 0; // For debug output timing

    while (1)
    {
        unsigned long now = micros();
        float deltaTime = (now - lastTime) / 1e6f;

        if (deltaTime < 0.001f)
        { // Minimum 1ms update rate
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        lastTime = now;

        long currentCount;
        portENTER_CRITICAL(&encMux);
        currentCount = encoderCount;
        portEXIT_CRITICAL(&encMux);

        // Track target position changes
        if (target_z_Pos != lastTarget)
        {
            lastTarget = target_z_Pos;
            movement_z_done = false;
            Serial.println("Z target changed to: " + String(target_z_Pos) + " mm");
        }

        float currentDistance = computeDistanceMM(currentCount);
        float currentVelocity = computeDistanceMM(currentCount - lastCount) / deltaTime; // Convert to mm/s
        lastCount = currentCount;

        // outer loop displacement control
        float targetVelocity = computePID(posPID, target_z_Pos, currentDistance, deltaTime);
        // inner loop: velocity control
        int pwm = (int)computePID(velPID, targetVelocity, currentVelocity, deltaTime);

        // Debug output every 500ms
        if (now - lastDebugTime >= 500000) // 500ms in microseconds
        {
            Serial.print("Z Pos: ");
            Serial.print(currentDistance, 2);
            Serial.print(" mm | Target: ");
            Serial.print(target_z_Pos, 2);
            Serial.print(" mm | Vel: ");
            Serial.print(currentVelocity, 2);
            Serial.print(" mm/s | PWM: ");
            Serial.println(pwm);
            lastDebugTime = now;
        }

        // if error smaller than 0.5mm stop
        if (abs(target_z_Pos - currentDistance) <= 0.5f)
        {
            Serial.println("Z position reached: " + String(currentDistance) + " mm");
            pwm = 0;
            movement_z_done = true;
        }
        setMotor(pwm);
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz update rate
    }
}

void startup_Z()
{
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(limitSwitchPin_z, INPUT);
    pinMode(ENC_A, INPUT); // use INPUT; use external pull-ups if needed
    pinMode(ENC_B, INPUT);

    // setup Enable PWM for enable pin
    setupPWM();

    // Attach single interrupt on A (use RISING to reduce noise; CHANGE is also OK if you want double resolution)
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

    lastTime = micros();

    Serial.println("Ready: PID position control (non-blocking).");
}
