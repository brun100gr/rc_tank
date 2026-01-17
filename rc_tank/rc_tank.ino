#include <Bluepad32.h>

// ===== TB6612 PINS =====
#define AIN1 18
#define AIN2 19
#define PWMA 23

#define BIN1 25
#define BIN2 26
#define PWMB 27

#define STBY 14

// ===== PWM CHANNELS =====
#define PWM_LEFT  0
#define PWM_RIGHT 1
#define PWM_FREQ  1000
#define PWM_RES   8   // 0–255

ControllerPtr controller = nullptr;

// --------------------------------------------------
// Utility: controlla un motore
// --------------------------------------------------
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

// --------------------------------------------------
// Tank drive: 2 joystick indipendenti
// --------------------------------------------------
void tankDrive(int left, int right) {
    setMotor(AIN1, AIN2, PWM_LEFT,  left);
    setMotor(BIN1, BIN2, PWM_RIGHT, right);
}

// --------------------------------------------------
// CALLBACKS Bluepad32
// --------------------------------------------------
void onConnectedController(ControllerPtr ctl) {
    Serial.println("Controller connesso");
    controller = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
    Serial.println("Controller disconnesso");
    controller = nullptr;
    tankDrive(0, 0);  // sicurezza: stop
}

// --------------------------------------------------
void setup() {
    Serial.begin(115200);

    // TB6612 setup
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    ledcSetup(PWM_LEFT,  PWM_FREQ, PWM_RES);
    ledcSetup(PWM_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWMA, PWM_LEFT);
    ledcAttachPin(PWMB, PWM_RIGHT);

    // Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // SOLO per debug iniziale
    BP32.forgetBluetoothKeys();

    Serial.println("Pronto: metti il controller in pairing");
}

// --------------------------------------------------
void loop() {
    BP32.update();

    if (controller && controller->isConnected()) {

        // Assi joystick
        int leftY  = controller->axisY();   // joystick sinistro Y
        int rightY = controller->axisRY();  // joystick destro Y

        // Invertiamo Y (su molti controller su = negativo)
        leftY  = -leftY;
        rightY = -rightY;

        // Mappa -511..512 → -255..255
        int leftSpeed  = map(leftY,  -512, 512, -255, 255);
        int rightSpeed = map(rightY, -512, 512, -255, 255);

        // Deadzone
        if (abs(leftSpeed)  < 20) leftSpeed  = 0;
        if (abs(rightSpeed) < 20) rightSpeed = 0;

        tankDrive(leftSpeed, rightSpeed);
    }

    delay(10);
}
