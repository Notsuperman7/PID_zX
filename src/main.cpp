#include <Arduino.h>

#define IN1 26
#define IN2 25
#define ENA 27
#define ENC_A 34
#define ENC_B 35
#define relay_pin 23

portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

volatile long encoderCount = 0;

const int PPR = 374;              // pulses per revolution (as you defined)
const float screw_lead = 0.8;    // linear cm per revolution (screw pitch)

// PID struct
struct PID {
  float Kp, Ki, Kd;
  float integral, prevError;
  float outMin, outMax;
};

// Tune these later if needed
PID posPID = {30, 0.5, 5.3, 0.0, 0.0, -200.0, 200.0};
PID velPID = {2, 0.1, 0.0, 0.0, 0.0, -255.0, 255.0};

float z_Distance = 5.0;    // target in cm
float currentDistance = 0.0;
float velocity = 0.0;
float vel_ref = 0.0;

unsigned long lastTime = 0;
long lastCount = 0;
const float dt = 0.01;      // 10 ms control loop

// ---------------- Encoder ISR ----------------
// single interrupt on ENC_A, sample ENC_B inside ISR
void IRAM_ATTR encoderISR() {
  // read both lines quickly
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);

  portENTER_CRITICAL_ISR(&encMux);
  // standard quadrature decoding (choose sign that matches your wiring)
  if (a == b) encoderCount--;  // direction sign chosen to match your wiring earlier
  else encoderCount++;
  portEXIT_CRITICAL_ISR(&encMux);
}

// ---------------- PID compute ----------------
float computePID(PID &pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  pid.integral += 0.5f * (error + pid.prevError) * dt;   // trapezoidal integration
  pid.integral = constrain(pid.integral, pid.outMin, pid.outMax);
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;
  float out = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  return constrain(out, pid.outMin, pid.outMax);
}

// Utility
float computeDistanceCM() {
  long c;
  portENTER_CRITICAL(&encMux);
  c = encoderCount;
  portEXIT_CRITICAL(&encMux);

  float rev = c / (float)PPR;   // revolutions
  return rev * screw_lead;     // linear cm
}

void setupPWM() {
  ledcSetup(0, 20000, 8);   // channel 0, 20kHz, 8-bit
  ledcAttachPin(ENA, 0);    //attach ch 0 to ENA pin
  ledcWrite(0, 0);
}

// IMPORTANT: mapping chosen so MOTOR DIR = 1 => DOWN (your requested behavior)
void setMotor(int pwm, int direction) {
  ledcWrite(0, pwm);
  if (direction == -1) {
    // DOWN
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if (direction == 1) {
    // UP
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void on_relay()  { digitalWrite(relay_pin, HIGH); }
void off_relay() { digitalWrite(relay_pin, LOW);  }

void setup() {
  Serial.begin(115200);

  pinMode(relay_pin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA,OUTPUT);

  pinMode(ENC_A, INPUT);    // use INPUT; use external pull-ups if needed
  pinMode(ENC_B, INPUT);

  // Attach single interrupt on A (use RISING to reduce noise; CHANGE is also OK if you want double resolution)
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  setupPWM();
  lastTime = micros();
  currentDistance = computeDistanceCM();

  Serial.println("Ready: PID position control (non-blocking).");
}

// ---------------- Main loop ----------------
void loop() {
  unsigned long now = micros();
  float delta = (now - lastTime) / 1e6f;
  if (delta < dt) return;
  lastTime = now;

  // read encoder safely
  long count;
  portENTER_CRITICAL(&encMux);
  count = encoderCount;
  portEXIT_CRITICAL(&encMux);

  // velocity in counts/sec
  velocity = (count - lastCount) / delta;
  lastCount = count;

  // position in cm
  currentDistance = computeDistanceCM();

  // outer loop: position -> velocity reference
  float pos_out = computePID(posPID, z_Distance, currentDistance, delta);
  vel_ref = constrain(pos_out, -300.0f, 300.0f);

  // inner loop: velocity control
  float u = computePID(velPID, vel_ref, velocity, delta);

  // pwm and direction from u
  int pwm = (int)fabs(u);
  pwm = constrain(pwm, 0, 200);
  int motorDir = (u > 0.1f) ? 1 : (u < -0.1f) ? -1 : 0;   // POS u -> DOWN (1)

  // deadband / arrived
  if (currentDistance >= 50.0) {
    pwm = 0;
    motorDir = 0;
    on_relay();
  } else {
    off_relay();
  }

  // apply motor command (non-blocking)
  setMotor(pwm, motorDir);

  // Serial debug
  Serial.print("Target(cm):"); Serial.print(z_Distance);
  Serial.print("  Pos(cm):"); Serial.print(currentDistance, 2);
  Serial.print("  Vel(cnt/s):"); Serial.print(velocity, 2);
  Serial.print("  VelRef:"); Serial.print(vel_ref, 2);
  Serial.print("  PWM:"); Serial.print(pwm);
  Serial.print("  DIR:"); Serial.println(motorDir);

  // monitor input to set new target
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.startsWith("set ")) {
      z_Distance = s.substring(4).toFloat();
      Serial.print("New z Distance = ");
      Serial.println(z_Distance);
      // optional: reset integrators to avoid bump
      posPID.integral = 0;
      velPID.integral = 0;
    }
  }
}
