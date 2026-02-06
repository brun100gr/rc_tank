#include <Wire.h>

// Imposta i pin I2C (modificali se necessario)
#define I2C_SDA 21
#define I2C_SCL 22

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nI2C Scanner ESP32");

  // Inizializza I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  scanI2C();
}

void loop() {
  // Nulla da fare nel loop
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scansione in corso...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C trovato all'indirizzo 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Errore sconosciuto all'indirizzo 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("Nessun dispositivo I2C trovato\n");
  else
    Serial.println("Scansione completata\n");
}
