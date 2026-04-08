#include <Wire.h>
#include <Arduino.h>

#define SDA_PIN 32
#define SCL_PIN 33

#define MPU6050_ADDR 0x68
const uint8_t addr = 0x68;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

void setup() {
  Serial.begin(115200);

  // I2C starten mit deinen Pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // MPU6050 aufwecken (Power Management Register)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Register
  Wire.write(0);     // Wake up
  Wire.endTransmission(true);

  Serial.println("MPU6050 bereit!");
}

void loop() {
  // Ab Adresse 0x3B lesen (Accel + Gyro)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(addr, (size_t)14, true);

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // Temperatur ignorieren

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  // Ausgabe
  Serial.print("Accel: ");
  Serial.print(accelX); Serial.print(" | ");
  Serial.print(accelY); Serial.print(" | ");
  Serial.print(accelZ);

  Serial.print("   Gyro: ");
  Serial.print(gyroX); Serial.print(" | ");
  Serial.print(gyroY); Serial.print(" | ");
  Serial.println(gyroZ);

  delay(500);
}