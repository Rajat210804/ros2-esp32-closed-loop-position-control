/*
 * Single-axis closed-loop position controller for ESP32.
 *
 * Geared DC motor + quadrature encoder + H-bridge + limit switch.
 * The PID loop runs here at a fixed 500 Hz. ROS only sends setpoints and
 * reads telemetry -- it is never inside the loop, because Linux scheduling
 * and USB latency cannot hold a position tolerance.
 *
 * Wiring (adjust the pin defines to match your board):
 *
 *   ENC_A     GPIO 34   encoder channel A   (input only pin, needs external pullup)
 *   ENC_B     GPIO 35   encoder channel B   (input only pin, needs external pullup)
 *   MOTOR_IN1 GPIO 25   H-bridge input 1
 *   MOTOR_IN2 GPIO 26   H-bridge input 2
 *   MOTOR_EN  GPIO 27   H-bridge enable / PWM
 *   LIMIT     GPIO 32   limit switch, wired normally-closed to GND
 *
 * Wire the limit switch normally CLOSED. A broken wire then reads the same as
 * a triggered switch and the axis refuses to move, instead of driving happily
 * into its own end stop. This is the single most important decision in the
 * file.
 */

#include <Arduino.h>

// ---------------------------------------------------------------- pins ----
constexpr int PIN_ENC_A     = 34;
constexpr int PIN_ENC_B     = 35;
constexpr int PIN_MOTOR_IN1 = 25;
constexpr int PIN_MOTOR_IN2 = 26;
constexpr int PIN_MOTOR_EN  = 27;
constexpr int PIN_LIMIT     = 32;

// ------------------------------------------------------------- tuning ----
constexpr uint32_t LOOP_HZ        = 500;
constexpr uint32_t LOOP_US        = 1000000UL / LOOP_HZ;
constexpr uint32_t TELEMETRY_HZ   = 50;
constexpr uint32_t TELEMETRY_US   = 1000000UL / TELEMETRY_HZ;

constexpr int   PWM_CHANNEL       = 0;
constexpr int   PWM_FREQ          = 20000;   // above audible, avoids motor whine
constexpr int   PWM_BITS          = 8;       // 0..255
constexpr int   PWM_MAX           = 255;

constexpr int   HOMING_PWM        = 90;      // slow, so the switch is not overrun
constexpr long  SOFT_LIMIT_MIN    = 0;
constexpr long  SOFT_LIMIT_MAX    = 40000;
constexpr float I_LIMIT           = 4000.0f;

// -------------------------------------------------------------- state ----
volatile long  g_position = 0;               // encoder counts, ISR-written
volatile uint8_t g_prevState = 0;

long   g_target   = 0;
float  g_kp = 4.0f, g_ki = 0.5f, g_kd = 0.12f;
float  g_integral = 0.0f;
long   g_lastMeasurement = 0;
bool   g_enabled = false;
bool   g_homed   = false;
int    g_pwm     = 0;

uint32_t g_lastLoopUs = 0;
uint32_t g_lastTelemetryUs = 0;

char   g_rxBuf[64];
uint8_t g_rxLen = 0;

// ------------------------------------------------------------ encoder ----
/*
 * x4 quadrature decoding via a 4-entry state transition table. Reading both
 * channels on every edge of either channel gives four counts per encoder
 * cycle, which is where the *4 in counts_per_rev comes from.
 */
void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;

  // Gray-code transition: XOR of the previous B with the current A gives
  // direction. Equivalent to a lookup table but branch-free and ISR-cheap.
  uint8_t prevB = g_prevState & 0x01;
  if (prevB ^ a) {
    g_position++;
  } else {
    g_position--;
  }
  g_prevState = state;
}

// -------------------------------------------------------------- motor ----
void motorWrite(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  g_pwm = pwm;

  if (pwm >= 0) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
  } else {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
    pwm = -pwm;
  }
  ledcWrite(PWM_CHANNEL, pwm);
}

void motorStop() {
  motorWrite(0);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
}

bool limitTriggered() {
  // Normally closed to GND: LOW means healthy, HIGH means triggered or broken.
  return digitalRead(PIN_LIMIT) == HIGH;
}

// ---------------------------------------------------------------- PID ----
void resetPID() {
  g_integral = 0.0f;
  noInterrupts();
  g_lastMeasurement = g_position;
  interrupts();
}

int computePID(long target, long measurement, float dt) {
  float error = (float)(target - measurement);

  // Derivative on measurement, not on error: a step change in the setpoint
  // then produces no derivative spike (no "derivative kick").
  float derivative = (float)(measurement - g_lastMeasurement) / dt;
  g_lastMeasurement = measurement;

  /*
   * Conditional integration (anti-windup).
   *
   * On a long move the output sits at the PWM limit for most of the travel.
   * If the integral keeps accumulating through that saturated stretch it
   * reaches a huge value, and the axis then overshoots badly while the
   * integral unwinds. Simply clamping the integral only caps how bad that
   * gets. The real fix is to freeze integration whenever the controller is
   * already saturated and the error would push it further in.
   */
  float candidate = g_integral + error * dt;
  float provisional = g_kp * error + g_ki * candidate - g_kd * derivative;

  bool inLinearRange = (provisional > -(float)PWM_MAX) && (provisional < (float)PWM_MAX);
  bool errorReversed = (provisional >= (float)PWM_MAX && error < 0.0f) ||
                       (provisional <= -(float)PWM_MAX && error > 0.0f);
  if (inLinearRange || errorReversed) {
    g_integral = candidate;
  }
  g_integral = constrain(g_integral, -I_LIMIT, I_LIMIT);

  float effort = g_kp * error + g_ki * g_integral - g_kd * derivative;
  return (int)constrain(effort, -(float)PWM_MAX, (float)PWM_MAX);
}

// -------------------------------------------------------------- homing ---
/*
 * Blocking homing routine: back off if already on the switch, seek toward it
 * slowly, then zero. Blocking is acceptable here because nothing else may
 * happen during homing anyway, and it keeps the state machine readable.
 */
void runHoming() {
  Serial.println("A homing: start");
  g_enabled = false;
  resetPID();

  // If we are sitting on the switch, walk off it first.
  uint32_t start = millis();
  while (limitTriggered() && millis() - start < 5000) {
    motorWrite(HOMING_PWM);
    delay(5);
  }
  motorStop();
  delay(150);

  // Seek toward the switch.
  start = millis();
  while (!limitTriggered()) {
    if (millis() - start > 20000) {
      motorStop();
      Serial.println("X homing: timeout, no limit switch found");
      return;
    }
    motorWrite(-HOMING_PWM);
    delay(2);
  }
  motorStop();
  delay(250);

  noInterrupts();
  g_position = 0;
  interrupts();

  g_target = 0;
  g_homed = true;
  g_enabled = true;
  resetPID();
  Serial.println("A homing: complete, zero set");
}

// ------------------------------------------------------------ protocol ---
void handleLine(char *line) {
  switch (line[0]) {
    case 'H':
      runHoming();
      break;

    case 'G': {
      long value = atol(line + 1);
      if (!g_homed) {
        Serial.println("X refused: axis not homed");
        break;
      }
      g_target = constrain(value, SOFT_LIMIT_MIN, SOFT_LIMIT_MAX);
      break;
    }

    case 'K': {
      float kp, ki, kd;
      if (sscanf(line + 1, "%f %f %f", &kp, &ki, &kd) == 3) {
        g_kp = kp; g_ki = ki; g_kd = kd;
        g_integral = 0.0f;
        Serial.print("A gains "); Serial.print(kp); Serial.print(" ");
        Serial.print(ki); Serial.print(" "); Serial.println(kd);
      } else {
        Serial.println("X bad gain command");
      }
      break;
    }

    case 'E':
      g_enabled = (atoi(line + 1) != 0);
      if (!g_enabled) motorStop();
      resetPID();
      Serial.println(g_enabled ? "A enabled" : "A disabled");
      break;

    case 'S':
      noInterrupts();
      g_target = g_position;
      interrupts();
      resetPID();
      Serial.println("A stopped");
      break;

    default:
      Serial.println("X unknown command");
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_rxLen > 0) {
        g_rxBuf[g_rxLen] = '\0';
        handleLine(g_rxBuf);
        g_rxLen = 0;
      }
    } else if (g_rxLen < sizeof(g_rxBuf) - 1) {
      g_rxBuf[g_rxLen++] = c;
    }
  }
}

// ---------------------------------------------------------------- main ---
void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  pinMode(PIN_LIMIT, INPUT_PULLUP);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_MOTOR_EN, PWM_CHANNEL);
  motorStop();

  g_prevState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  g_lastLoopUs = micros();
  Serial.println("A single-axis controller ready, send H to home");
}

void loop() {
  pollSerial();

  uint32_t now = micros();

  // ---- control loop, fixed rate ----
  if (now - g_lastLoopUs >= LOOP_US) {
    float dt = (now - g_lastLoopUs) * 1e-6f;
    g_lastLoopUs = now;

    noInterrupts();
    long position = g_position;
    interrupts();

    if (g_enabled && g_homed) {
      if (limitTriggered() && g_target < position) {
        // Never drive further into a triggered end stop.
        motorStop();
        g_target = position;
        resetPID();
      } else {
        motorWrite(computePID(g_target, position, dt));
      }
    } else if (!g_enabled) {
      motorStop();
    }
  }

  // ---- telemetry, fixed rate ----
  if (now - g_lastTelemetryUs >= TELEMETRY_US) {
    g_lastTelemetryUs = now;

    noInterrupts();
    long position = g_position;
    interrupts();

    Serial.print("T ");
    Serial.print(now);            Serial.print(' ');
    Serial.print(position);       Serial.print(' ');
    Serial.print(g_target);       Serial.print(' ');
    Serial.print(g_target - position); Serial.print(' ');
    Serial.print(g_pwm);          Serial.print(' ');
    Serial.println(g_homed ? 1 : 0);
  }
}
