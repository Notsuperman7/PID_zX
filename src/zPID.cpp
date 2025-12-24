#include <Arduino.h>
#include "config_Z.h"
#include "homing_flags.h"

/* ====== Global targets/flags ====== */
float target_z_Pos = 0.0f;               // mm
volatile bool movement_z_done = true;
volatile bool homingDone_z = false;

/* ====== Encoder state ====== */
volatile long encoderCount = 0;
volatile uint32_t isrCallCount = 0;

unsigned long lastTime = 0;
long lastCount = 0;
// ===== MATLAB tuned gains (Z axis) =====
constexpr float Kp_pos = 42.8323f;   // 1/s  (v_sp = Kp_pos * pos_error)
constexpr float Kp_vel = 2.7956f;   // PWM / (mm/s)
constexpr float Ki_vel = 1.2809f;   // PWM / mm   (integral of velocity error)

// ===== Limits / safety =====
constexpr int PWM_MIN = -220;
constexpr int PWM_MAX = 220;

constexpr float VMAX   = 100.0f;    // mm/s  (safe start)
constexpr float POS_TOL = 0.5f;     // mm (stop band)


/* ---------------- Encoder ISR ---------------- */
void IRAM_ATTR encoderISR()
{
    isrCallCount++;
    uint32_t gpioState = REG_READ(GPIO_IN_REG);
    uint8_t b = (gpioState >> ENC_B) & 1;

    // rising edge on A: direction by B
    if (b) encoderCount--;
    else   encoderCount++;
}

/* ---------------- Motor ---------------- */
void setMotor(int pwm)
{
    //pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    analogWrite(ENA, abs(pwm));

    if (pwm > 0) {
        // DOWN
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    } else if (pwm < 0) {
        // UP
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
}

/* ---------------- Units conversion ---------------- */
static inline float computeDistanceMM(long count)
{
    // count -> rev -> mm
    float rev = (count / (float)PPR);
    return rev * screw_lead;
}

/* ---------------- Homing task ---------------- */
void home_z(void *pvParameters)
{
    Serial.println("Homing z axis...");
    setMotor(-70);
    vTaskDelay(pdMS_TO_TICKS(50));

    while (digitalRead(limitSwitchPin_z) == HIGH) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    setMotor(0);
    vTaskDelay(pdMS_TO_TICKS(50));
    setMotor(120);
    while (digitalRead(limitSwitchPin_z) != HIGH) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    setMotor(0);
    
    encoderCount = 0;
    isrCallCount = 0;

    homingDone_z = true;
    Serial.println("Z axis homed to position 0");
    vTaskDelete(NULL);
}

/* ---------------- Main cascaded control ---------------- */
void applyPID(void *parameter)
{
    while (!homingDone_x || !homingDone_y || !homingDone_z) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println("Starting Z cascaded control: Position(P) + Velocity(PI)");

    lastTime = micros();
    lastCount = encoderCount;

    float velIntegral = 0.0f;          // integral of velocity error
    float lastTarget = -999.0f;
    unsigned long lastDebugTime = 0;

    while (1)
    {
        unsigned long now = micros();
        float dt = (now - lastTime) * 1e-6f;

        if (dt < 0.001f) { // >= 1ms
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        lastTime = now;

        long currentCount = encoderCount;

        // detect target change
        if (target_z_Pos != lastTarget) {
            lastTarget = target_z_Pos;
            movement_z_done = false;
            Serial.println("Z target changed to: " + String(target_z_Pos, 2) + " mm");
        }

        // Measurements (mm, mm/s)
        float pos_mm = computeDistanceMM(currentCount);
        float vel_mm_s = computeDistanceMM(currentCount - lastCount) / dt;
        lastCount = currentCount;

        // -------- Outer loop: Position P (produces velocity setpoint) --------
        float e_pos = target_z_Pos - pos_mm;                 // mm
        float v_sp = Kp_pos * e_pos;                         // mm/s
        v_sp = constrain(v_sp, -VMAX, VMAX);                 // clamp

        // -------- Inner loop: Velocity PI (produces PWM) --------
        float e_vel = v_sp - vel_mm_s;                       // mm/s

        // candidate integrator update
        float velIntegral_candidate = velIntegral + e_vel * dt;

        // unsaturated control
        float u_unsat = Kp_vel * e_vel + Ki_vel * velIntegral_candidate;

        // saturate
        float u_sat = constrain(u_unsat, (float)PWM_MIN, (float)PWM_MAX);

        // Anti-windup (simple): only accept integrator update if not saturating
        // or if the error would drive the output back from saturation.
        bool saturatingHigh = (u_unsat > PWM_MAX);
        bool saturatingLow  = (u_unsat < PWM_MIN);

        if ((!saturatingHigh && !saturatingLow) ||
            (saturatingHigh && e_vel < 0) ||
            (saturatingLow  && e_vel > 0))
        {
            velIntegral = velIntegral_candidate;
        }

        int pwm = (int)u_sat;

        // stop condition near target
        if (fabs(e_pos) <= POS_TOL) {
            pwm = 0;
            movement_z_done = true;
        }

        setMotor(pwm);

        // Debug every 500ms
        if (now - lastDebugTime >= 500000) {
            Serial.print("Z Pos: "); Serial.print(pos_mm, 2);
            Serial.print(" mm | Target: "); Serial.print(target_z_Pos, 2);
            Serial.print(" mm | Vel: "); Serial.print(vel_mm_s, 2);
            Serial.print(" mm/s | v_sp: "); Serial.print(v_sp, 2);
            Serial.print(" | PWM: "); Serial.print(pwm);
            Serial.print(" | ISR: "); Serial.println(isrCallCount);
            lastDebugTime = now;
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz
    }
}

/* ---------------- Startup ---------------- */
void startup_Z()
{
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    analogWrite(ENA, 0);

    pinMode(limitSwitchPin_z, INPUT_PULLUP);
    pinMode(ENC_A, INPUT_PULLDOWN);
    pinMode(ENC_B, INPUT_PULLDOWN);

    //setupPWM();

    int intNum = digitalPinToInterrupt(ENC_A);
    if (intNum == NOT_AN_INTERRUPT) {
        Serial.println("ERROR: GPIO pin does not support interrupts!");
    } else {
        attachInterrupt(intNum, encoderISR, RISING);
        Serial.println("Interrupt attached on ENC_A RISING");
    }

    lastTime = micros();
    Serial.println("Ready: Z cascaded control (P pos + PI vel).");
}
