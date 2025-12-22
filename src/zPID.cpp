#include <Arduino.h>
#include "config_Z.h"
#include "homing_flags.h"

float target_z_Pos = 00.0; // target in mm
volatile bool movement_z_done = true;
volatile bool homingDone_z = false;

volatile long encoderCount = 0;
volatile uint8_t lastEncoderState = 0; // Store last encoder state for quadrature decoding
volatile uint32_t isrCallCount = 0;    // Debug: count ISR calls

unsigned long lastTime = 0;
long lastCount = 0;
// No lookup table needed - simple direction detection on RISING edge of A
// Direction determined by B phase when A rises

// ---------------- Encoder ISR ----------------
// Interrupt on RISING edge of ENC_A only - more reliable for fast movement
void IRAM_ATTR encoderISR()
{
    isrCallCount++; // Debug counter

    // Read both lines using ESP32 GPIO registers
    uint32_t gpioState = REG_READ(GPIO_IN_REG);
    uint8_t a = (gpioState >> ENC_A) & 1;
    uint8_t b = (gpioState >> ENC_B) & 1;

    // On rising edge of A: if B is low, we're going forward; if B is high, backward
    if (b)
    {
        encoderCount--; // Backward
    }
    else
    {
        encoderCount++; // Forward
    }
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
    setMotor(-180);
    vTaskDelay(pdMS_TO_TICKS(50)); // Let motor start
    while (digitalRead(limitSwitchPin_z) == HIGH)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    setMotor(0);
    encoderCount = 0; // Reset encoder count - atomic assignment, no critical section needed
    isrCallCount = 0; // Reset ISR call counter for diagnostics

    homingDone_z = true;
    Serial.println("Z axis homed to position 0");

    vTaskDelete(NULL);
}

inline float computeDistanceMM(long count)
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

        // Read encoder count - no critical section needed for single atomic read on ESP32
        long currentCount = encoderCount;
        uint32_t currentIsrCalls = isrCallCount;

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
            Serial.print(" mm | Count: ");
            Serial.print(currentCount);
            Serial.print(" | ISR calls: ");
            Serial.print(currentIsrCalls);
            Serial.print(" | Vel: ");
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
    // Enable internal pull-ups on encoder lines to avoid floating inputs
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    // setup Enable PWM for enable pin
    setupPWM();

    // Initialize last encoder state from the pins before enabling interrupts
    {
        uint32_t gpioState = REG_READ(GPIO_IN_REG);
        bool a = (gpioState >> ENC_A) & 1;
        bool b = (gpioState >> ENC_B) & 1;
        lastEncoderState = (a << 1) | b;
        Serial.print("Initial encoder state - A(GPIO");
        Serial.print(ENC_A);
        Serial.print("):");
        Serial.print(a);
        Serial.print(" B(GPIO");
        Serial.print(ENC_B);
        Serial.print("):");
        Serial.println(b);
    }

    // Attach interrupt on RISING edge of ENC_A only - simpler and more reliable
    Serial.print("Attempting to attach interrupt to GPIO");
    Serial.print(ENC_A);
    Serial.println("...");

    int intNum = digitalPinToInterrupt(ENC_A);
    Serial.print("Interrupt number: ");
    Serial.println(intNum);

    if (intNum == NOT_AN_INTERRUPT)
    {
        Serial.println("ERROR: GPIO pin does not support interrupts!");
    }
    else
    {
        attachInterrupt(intNum, encoderISR, RISING);
        Serial.println("Interrupt attached successfully on RISING edge");
    }

    lastTime = micros();

    Serial.println("Ready: PID position control (non-blocking).");
}
