#include <Bluepad32.h>

// ==================================================
// STATUS LED
// ==================================================
#define LED_BUILTIN 2

// ==================================================
// TB6612 MOTOR DRIVER PINS
// ==================================================
#define AIN1 18
#define AIN2 19
#define PWMA 23

#define BIN1 25
#define BIN2 26
#define PWMB 27

#define STBY 14

// ==================================================
// PWM CONFIGURATION
// ==================================================
#define PWM_LEFT_CH   0
#define PWM_RIGHT_CH  1

#define PWM_FREQ   1000
#define PWM_RES    8

// ==================================================
// MT6701 PWM INPUT PINS
// ==================================================
#define ENC_LEFT_PIN   32
#define ENC_RIGHT_PIN  33

// ==================================================
// PID PARAMETERS (tuning)
// ==================================================
float Kp = 0.5;
float Ki = 0.1;
float Kd = 0.0;

float pidIntegral = 0;
float pidLastError = 0;
unsigned long pidLastTime = 0;

// ==================================================
// CONTROLLER POINTER
// ==================================================
ControllerPtr controller = nullptr;

// ==================================================
// MOTOR CONTROL
// ==================================================
void setMotor(int in1, int in2, int pwmChannel, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, -speed);
  } else {
    ledcWrite(pwmChannel, 0);
  }
}

void tankDrive(int left, int right) {
  setMotor(AIN1, AIN2, PWM_LEFT_CH, left);
  setMotor(BIN1, BIN2, PWM_RIGHT_CH, right);
}

// ==================================================
// MT6701 PWM → ANGLE
// ==================================================
float readAnglePWM(int pin) {
  unsigned long highT = pulseIn(pin, HIGH, 2000);
  unsigned long lowT  = pulseIn(pin, LOW, 2000);

  if (highT == 0 || lowT == 0) return NAN;

  float duty = (float)highT / (highT + lowT);
  return duty * 360.0;
}

float deltaAngle(float now, float prev) {
  float d = now - prev;
  if (d > 180)  d -= 360;
  if (d < -180) d += 360;
  return d;
}

// ==================================================
// PID – wheel sync
// ==================================================
float wheelSyncPID(float speedL, float speedR) {
  unsigned long now = millis();
  float dt = (now - pidLastTime) / 1000.0;
  if (dt <= 0) return 0;

  float error = speedL - speedR;
  pidIntegral += error * dt;
  float derivative = (error - pidLastError) / dt;

  float output = Kp * error + Ki * pidIntegral + Kd * derivative;

  pidLastError = error;
  pidLastTime = now;

  return output;
}

// ==================================================
// BLUETOOTH CALLBACKS
// ==================================================
void onConnectedController(ControllerPtr ctl) {
  controller = ctl;
  digitalWrite(LED_BUILTIN, HIGH);
}

void onDisconnectedController(ControllerPtr ctl) {
  controller = nullptr;
  digitalWrite(LED_BUILTIN, LOW);
  tankDrive(0, 0);
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcSetup(PWM_LEFT_CH,  PWM_FREQ, PWM_RES);
  ledcSetup(PWM_RIGHT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_LEFT_CH);
  ledcAttachPin(PWMB, PWM_RIGHT_CH);

  pinMode(ENC_LEFT_PIN, INPUT);
  pinMode(ENC_RIGHT_PIN, INPUT);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  Serial.println("RC Tank ready (PID wheel sync enabled)");
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  BP32.update();

  static float lastAngleL = 0;
  static float lastAngleR = 0;
  static unsigned long lastTime = millis();

  if (!controller || !controller->isConnected()) return;

  int joyL = map(-controller->axisY(),  -512, 512, -255, 255);
  int joyR = map(-controller->axisRY(), -512, 512, -255, 255);

  if (abs(joyL) < 20) joyL = 0;
  if (abs(joyR) < 20) joyR = 0;

  // === encoder speed ===
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  float angleL = readAnglePWM(ENC_LEFT_PIN);
  float angleR = readAnglePWM(ENC_RIGHT_PIN);

  float speedL = 0;
  float speedR = 0;

  if (!isnan(angleL) && !isnan(angleR) && dt > 0) {
    speedL = deltaAngle(angleL, lastAngleL) / dt;
    speedR = deltaAngle(angleR, lastAngleR) / dt;
    lastAngleL = angleL;
    lastAngleR = angleR;
  }

  lastTime = now;

  // === CURVE DETECTION ===
  int diffJoy = abs(joyL - joyR);
  float straightFactor = 1.0 - constrain(diffJoy / 255.0, 0.0, 1.0);

  float correction = wheelSyncPID(speedL, speedR) * straightFactor;
  correction = constrain(correction, -50, 50);

  int motorL = joyL - correction;
  int motorR = joyR + correction;

  motorL = constrain(motorL, -255, 255);
  motorR = constrain(motorR, -255, 255);

  tankDrive(motorL, motorR);

  // === DEBUG (Serial Plotter friendly) ===
  Serial.print(speedL);
  Serial.print(",");
  Serial.print(speedR);
  Serial.print(",");
  Serial.println(correction);

  delay(20);
}
