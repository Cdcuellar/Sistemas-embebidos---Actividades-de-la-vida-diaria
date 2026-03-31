#include <Wire.h>

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);

  Serial.println("Leyendo WHO_AM_I...");
  
  Wire.beginTransmission(0x68);
  Wire.write(0x75); // Registro WHO_AM_I
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 1);

  if (Wire.available()) {
    uint8_t val = Wire.read();
    Serial.print("WHO_AM_I: 0x");
    Serial.println(val, HEX);
  } else {
    Serial.println("No se pudo leer el sensor");
  }
}

void loop() {}