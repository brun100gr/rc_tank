#include <Wire.h>

// ==================================================
// MT6701 CONFIG
// ==================================================
#define MT6701_ADDR        0x06    // prova anche 0x36 se non risponde
#define MT6701_ANGLE_REG   0x03    // MSB angolo
#define MT6701_STATUS_REG  0x0B    // registro stato magnete
#define ENCODER_RESOLUTION 16384   // 14 bit

// ==================================================
// VARIABLES
// ==================================================
uint16_t lastAngle = 0;
unsigned long lastTime = 0;

float speed_ticks_s = 0.0;
float speed_rpm     = 0.0;

// ==================================================
// READ ANGLE
// ==================================================
uint16_t readMT6701Angle() {

    Wire.beginTransmission((uint8_t)MT6701_ADDR);
    Wire.write((uint8_t)MT6701_ANGLE_REG);
    if (Wire.endTransmission(false) != 0) {
        return 0;
    }

    Wire.requestFrom((uint8_t)MT6701_ADDR, (uint8_t)2);
    if (Wire.available() < 2) {
        return 0;
    }

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();

    uint16_t angle = ((uint16_t)msb << 8) | lsb;
    angle &= 0x3FFF;

    return angle;
}

// ==================================================
// READ STATUS REGISTER
// ==================================================
uint8_t readMT6701Status() {

    Wire.beginTransmission((uint8_t)MT6701_ADDR);
    Wire.write((uint8_t)MT6701_STATUS_REG);
    if (Wire.endTransmission(false) != 0) {
        return 0xFF;   // errore I2C
    }

    Wire.requestFrom((uint8_t)MT6701_ADDR, (uint8_t)1);
    if (Wire.available() < 1) {
        return 0xFF;
    }

    return Wire.read();
}

// ==================================================
// ANGLE DELTA WITH ROLLOVER
// ==================================================
int16_t computeAngleDelta(uint16_t current, uint16_t previous) {

    int16_t delta = current - previous;

    if (delta >  ENCODER_RESOLUTION / 2)
        delta -= ENCODER_RESOLUTION;
    else if (delta < -ENCODER_RESOLUTION / 2)
        delta += ENCODER_RESOLUTION;

    return delta;
}

// ==================================================
// SETUP
// ==================================================
void setup() {

    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("MT6701 encoder + magnet diagnostic");
    Serial.println("----------------------------------");

    Wire.begin();
    Wire.setClock(400000);

    lastAngle = readMT6701Angle();
    lastTime  = micros();
}

// ==================================================
// LOOP
// ==================================================
void loop() {

    unsigned long now = micros();
    unsigned long dt_us = now - lastTime;

    if (dt_us >= 5000) {   // ~5 ms

        uint16_t angle = readMT6701Angle();
        int16_t delta  = computeAngleDelta(angle, lastAngle);

        float dt = dt_us / 1e6;

        speed_ticks_s = delta / dt;
        speed_rpm     = speed_ticks_s * 60.0 / ENCODER_RESOLUTION;

        uint8_t status = readMT6701Status();

        // ===== SERIAL OUTPUT (human readable) =====
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print(" | RPM: ");
        Serial.print(speed_rpm, 2);
        Serial.print(" | Magnet: ");

        if (status == 0xFF) {
            Serial.println("I2C ERROR");
        } else if (status & 0x01) {
            Serial.println("NO MAGNET");
        } else if (status & 0x02) {
            Serial.println("TOO WEAK");
        } else if (status & 0x04) {
            Serial.println("TOO STRONG");
        } else {
            Serial.println("OK");
        }

        lastAngle = angle;
        lastTime  = now;
    }
}
