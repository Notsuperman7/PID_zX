#include <Arduino.h>

#define IN1 26
#define IN2 25
#define ENA 27
#define ENC_A 34
#define ENC_B 35
#define Lim_Z 1

portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

volatile long encoderCount = 0;

constexpr int PPR = 374;        // pulses per revolution (as you defined)
constexpr float screw_lead = 8; // linear mm per revolution (screw pitch)

float targetDistance = 50.0; // target in mm

unsigned long lastTime = 0;
long lastCount = 0;

// PID struct
struct PID
{
  float Kp, Ki, Kd;
  float integral, prevError;
  float outMin, outMax;
};

// Tune these later if needed
PID posPID = {30, 0.5, 5.3, 0.0, 0.0, -200.0, 200.0};
PID velPID = {2, 0.1, 0.0, 0.0, 0.0, -255.0, 255.0};

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

  ledcWrite(0, pwm);
  if (pwm > 0)
  {
    // DOWN
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if (pwm < 0)
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
void home_z()
{
  setMotor(-100);
  while (digitalRead(Lim_Z) == HIGH)
  {
  }
  setMotor(0);
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
  while (1)
  {
    unsigned long now = micros();
    float deltaTime = (now - lastTime) / 1e6f;

    lastTime = now;

    long currentCount;
    portENTER_CRITICAL(&encMux);
    currentCount = encoderCount;
    portEXIT_CRITICAL(&encMux);

    float currentVelocity = (currentCount - lastCount) / deltaTime;
    lastCount = currentCount;
    float currentDistance = computeDistanceMM(currentCount);

    // outer loop displacement control
    float targetVelocity = computePID(posPID, targetDistance, currentDistance, deltaTime);
    // inner loop: velocity control
    int pwm = (int)computePID(velPID, targetVelocity, currentVelocity, deltaTime);

    // if error smaller than 5% of targetDistance stop
    if (abs(targetDistance - currentDistance) <= 0.05 * targetDistance)
    {
      pwm = 0;
    }

    setMotor(pwm);
    delay(10);
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(Lim_Z, INPUT);
  pinMode(ENC_A, INPUT); // use INPUT; use external pull-ups if needed
  pinMode(ENC_B, INPUT);

  // setup Enable PWM for enable pin
  setupPWM();

  home_z();

  // Attach single interrupt on A (use RISING to reduce noise; CHANGE is also OK if you want double resolution)
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  lastTime = micros();

  Serial.println("Ready: PID position control (non-blocking).");

  xTaskCreate(
      applyPID,   // Function name of the task
      "applyPID", // Name of the task (e.g. for debugging)
      2048,       // Stack size (bytes)
      NULL,       // Parameter to pass
      1,          // Task priority
      NULL        // Task handle
  );
}

void loop()
{
}