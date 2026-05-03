// ======================
// MOTOR PINS (L298) — MEGA 2560 SAFE VERSION
#define IN1 30
#define IN2 31
#define ENA 9
#define IN3 32
#define IN4 33
#define ENB 11

// ======================
// ENCODER PINS
#define ENC_LEFT_A 18
#define ENC_LEFT_B 4
#define ENC_RIGHT_A 19
#define ENC_RIGHT_B 5

// ======================
#define TICKS_PER_REV 234.0
#define MAX_SPEED     20.0     // ← rad/s — your motor's working max speed
#define MAX_PWM       255.0
#define Kff           (MAX_PWM / MAX_SPEED)  // = 12.75 automatically
#define MIN_PWM       60 

volatile long ticks_left  = 0;
volatile long ticks_right = 0;
long prev_ticks_left  = 0;
long prev_ticks_right = 0;
unsigned long last_time    = 0;
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

// ======================
// PID GAINS
float Kp = 1.0;    // start low — feedforward does most of the work
float Ki = 0.0;    // small integral to fix steady-state error
float Kd = 0.0;    // keep zero until stable

float target_vel_left  = 0;
float target_vel_right = 0;
float integral_left    = 0;
float integral_right   = 0;
float prev_error_left  = 0;
float prev_error_right = 0;

// ======================
void leftEncoderISR() {
  bool A = digitalRead(ENC_LEFT_A);
  bool B = digitalRead(ENC_LEFT_B);
  if (A ^ B) ticks_left++;
  else       ticks_left--;
}

void rightEncoderISR() {
  bool A = digitalRead(ENC_RIGHT_A);
  bool B = digitalRead(ENC_RIGHT_B);
  if (A ^ B) ticks_right++;
  else       ticks_right--;
}

// ======================
float computePID(float target, float measured,
                 float &integral, float &prev_error, float dt) {
  float error      = target - measured;
  integral        += error * dt;
  integral         = constrain(integral, -50, 50);  // anti-windup
  float derivative = (error - prev_error) / dt;
  prev_error       = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

// ======================
void moveMotors(float pwmLeft, float pwmRight) {

  // Map PID output (0-255) to real motor range (MIN_PWM-255)
  // This avoids sudden jumps
  auto applyDeadZone = [](float pwm) -> int {
    if (fabs(pwm) < 1.0) return 0;   // true zero — stop motor
    // scale: small PID output → MIN_PWM, full PID output → 255
    int mapped = (int)(MIN_PWM + (fabs(pwm) / 255.0) * (255 - MIN_PWM));
    return constrain(mapped, MIN_PWM, 255);
  };

  int absLeft  = applyDeadZone(pwmLeft);
  int absRight = applyDeadZone(pwmRight);

  if      (pwmLeft  > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (pwmLeft  < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else                   { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }

  if      (pwmRight > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else if (pwmRight < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else                   { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }

  analogWrite(ENA, absLeft);
  analogWrite(ENB, absRight);
}
// ======================
void readCommand() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      target_vel_left  = input.substring(0, commaIndex).toFloat();
      target_vel_right = input.substring(commaIndex + 1).toFloat();
      last_cmd_time    = millis();

      if (target_vel_left  == 0) { integral_left  = 0; prev_error_left  = 0; }
      if (target_vel_right == 0) { integral_right = 0; prev_error_right = 0; }
    }
  }
}

// ======================
void sendFeedback(unsigned long now) {
  float dt = (now - last_time) / 1000.0;
  if (dt <= 0) return;

  noInterrupts();
  long t_left  = ticks_left;
  long t_right = ticks_right;
  interrupts();

  float pos_left  = -(t_left  / TICKS_PER_REV) * 2.0 * PI;
  float pos_right = -(t_right / TICKS_PER_REV) * 2.0 * PI;

  long d_left  = t_left  - prev_ticks_left;
  long d_right = t_right - prev_ticks_right;

  float vel_left  = -(d_left  / TICKS_PER_REV) * 2.0 * PI / dt;
  float vel_right = -(d_right / TICKS_PER_REV) * 2.0 * PI / dt;

  if (fabs(vel_left)  < 0.05) vel_left  = 0;
  if (fabs(vel_right) < 0.05) vel_right = 0;

  float pwm_left  = 0;
  float pwm_right = 0;

  if (target_vel_left != 0) {
    float ff_left  = Kff * target_vel_left;              // feedforward
    float pid_left = computePID(target_vel_left, vel_left,
                                integral_left, prev_error_left, dt);
    pwm_left = constrain(ff_left + pid_left, -255, 255); // combined
  }

  if (target_vel_right != 0) {
    float ff_right  = Kff * target_vel_right;
    float pid_right = computePID(target_vel_right, vel_right,
                                 integral_right, prev_error_right, dt);
    pwm_right = constrain(ff_right + pid_right, -255, 255);
  }

  moveMotors(pwm_left, pwm_right);

  prev_ticks_left  = t_left;
  prev_ticks_right = t_right;
  last_time        = now;

  Serial.print(pos_left,  4); Serial.print(",");
  Serial.print(pos_right, 4); Serial.print(",");
  Serial.print(vel_left,  4); Serial.print(",");
  Serial.println(vel_right, 4);
}

// ======================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  delay(500);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(ENC_LEFT_A,  INPUT_PULLUP); pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP); pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

  last_time     = millis();
  last_cmd_time = millis();
}

// ======================
void loop() {
  readCommand();
  unsigned long now = millis();

  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    target_vel_left = 0; target_vel_right = 0;
    integral_left   = 0; integral_right   = 0;
    moveMotors(0, 0);
  }

  if (now - last_time >= 50) {
    sendFeedback(now);
  }
}
