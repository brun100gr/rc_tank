#include <Bluepad32.h>   // Library that handles Bluetooth HID controllers (PS4, Xbox, etc.)


// ==================================================
// STATUS LED
// ==================================================
// Built-in LED on most ESP32 boards.
// We'll use it to indicate:
// - OFF  → no controller connected
// - ON   → controller connected
#define LED_BUILTIN 2


// ==================================================
// TB6612 MOTOR DRIVER PINS
// ==================================================
// Left motor
#define AIN1 18      // Left motor direction
#define AIN2 19      // Left motor direction
#define PWMA 23      // Left motor PWM speed

// Right motor
#define BIN1 25      // Right motor direction
#define BIN2 26      // Right motor direction
#define PWMB 27      // Right motor PWM speed

// TB6612 Standby: must be HIGH to enable motors
#define STBY 14


// ==================================================
// PWM CONFIGURATION (ESP32 LEDC)
// ==================================================
// We'll use two ESP32 hardware PWM channels
// - one for left motor
// - one for right motor
#define PWM_LEFT   0
#define PWM_RIGHT  1

#define PWM_FREQ   1000   // PWM frequency in Hz (1 kHz is good for DC motors)
#define PWM_RES    8      // Resolution: 8 bit → values 0–255


// ==================================================
// POINTER TO CONNECTED CONTROLLER
// ==================================================
// ControllerPtr is a pointer provided by Bluepad32.
// If nullptr → no controller connected.
ControllerPtr controller = nullptr;


// ==================================================
// UTILITY FUNCTION: controls ONE motor
// ==================================================
// in1 / in2      → direction pins
// pwmChannel    → associated PWM channel
// speed         → speed from -255 to +255
//
// speed > 0  → forward
// speed < 0  → backward
// speed = 0  → stop
void setMotor(int in1, int in2, int pwmChannel, int speed) {

    // Limit speed for safety
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        // Forward rotation
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        ledcWrite(pwmChannel, speed);
    }
    else if (speed < 0) {
        // Backward rotation
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        ledcWrite(pwmChannel, -speed); // positive value for PWM
    }
    else {
        // Motor stopped
        ledcWrite(pwmChannel, 0);
    }
}


// ==================================================
// TANK DRIVE
// ==================================================
// Tank-style control:
// - left joystick → left track
// - right joystick → right track
void tankDrive(int left, int right) {
    setMotor(AIN1, AIN2, PWM_LEFT,  left);
    setMotor(BIN1, BIN2, PWM_RIGHT, right);
}


// ==================================================
// BLUETOOTH CALLBACK: controller connected
// ==================================================
// This function is called AUTOMATICALLY
// by Bluepad32 when a controller connects.
void onConnectedController(ControllerPtr ctl) {
    Serial.println("Controller connected");

    // Save pointer to controller
    controller = ctl;

    // Turn on LED to indicate active connection
    digitalWrite(LED_BUILTIN, HIGH);
}


// ==================================================
// BLUETOOTH CALLBACK: controller disconnected
// ==================================================
void onDisconnectedController(ControllerPtr ctl) {
    Serial.println("Controller disconnected");

    // Reset pointer
    controller = nullptr;

    // LED off → no controller
    digitalWrite(LED_BUILTIN, LOW);

    // Safety: stop motors immediately
    tankDrive(0, 0);
}


// ==================================================
// SETUP
// ==================================================
void setup() {
    Serial.begin(115200);

    // ---------- LED ----------
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);   // LED off at startup

    // ---------- TB6612 ----------
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    // Enable motor driver
    digitalWrite(STBY, HIGH);

    // ---------- PWM ----------
    ledcSetup(PWM_LEFT,  PWM_FREQ, PWM_RES);
    ledcSetup(PWM_RIGHT, PWM_FREQ, PWM_RES);

    // Attach PWM pins to channels
    ledcAttachPin(PWMA, PWM_LEFT);
    ledcAttachPin(PWMB, PWM_RIGHT);

    // ---------- Bluepad32 ----------
    // Register connection/disconnection callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // ONLY for initial debug:
    // clear previous pairings
    BP32.forgetBluetoothKeys();

    Serial.println("Ready: put the controller in pairing mode");
}


// ==================================================
// MAIN LOOP
// ==================================================
void loop() {

    // Update Bluetooth status (MANDATORY)
    BP32.update();

    // Proceed only if a controller is connected
    if (controller && controller->isConnected()) {

        // Read Y joystick axes
        // Note: on many controllers "up" is negative → invert sign
        int rawLeftY  = -controller->axisY();   // left joystick
        int rawRightY = -controller->axisRY();  // right joystick

        // Map Bluepad32 range (-512..512)
        // to PWM range (-255..255)
        int leftSpeed  = map(rawLeftY,  -512, 512, -255, 255);
        int rightSpeed = map(rawRightY, -512, 512, -255, 255);

        // Deadzone to avoid jitter at rest position
        if (abs(leftSpeed)  < 20) leftSpeed  = 0;
        if (abs(rightSpeed) < 20) rightSpeed = 0;

        // Debug log: show input → output
        Serial.printf(
            "MAP | L:%4d -> %4d   R:%4d -> %4d\n",
            rawLeftY, leftSpeed,
            rawRightY, rightSpeed
        );

        // Command motors
        tankDrive(leftSpeed, rightSpeed);

        // Delay for:
        // - avoid serial spam
        // - avoid watchdog
        delay(100);
    }
}
