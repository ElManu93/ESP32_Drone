#include <Wire.h>
#include <Arduino.h>

#define SDA_PIN 32
#define SCL_PIN 33

const uint8_t MPU6050_ADDR = 0x68;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void setupMPU6050() {
  // MPU6050 aufwecken (Power Management Register)
  // Register 0x6B (PWR_MGMT_1)
  // Wake up (BIT6 can be set to put in sleep mode)
  writeRegister(MPU6050_ADDR, 0x6B, 0x00);

  // Set senitivities:
  // Accelrometer: (Register 0x1C)
  // BIT3 * BIT4 decide about sensitivity (0x08 == ±4g)
  writeRegister(MPU6050_ADDR, 0x1C, 0x08);

  // Accelrometer: (Register 0x1C)
  // BIT3 * BIT4 decide about sensitivity (0x08 == ±500 °/s)
  writeRegister(MPU6050_ADDR, 0x1B, 0x08);
  
  Serial.println("MPU6050 bereit!");
}

void getSensorData() {
  // Begin reading data from register 0x3B
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);  // Block I2C-BUS

  // Request 14 Bytes:
  Wire.requestFrom(MPU6050_ADDR, (size_t)14, true);

  // Get 16 Bit values for each reading
  // << 8 verschiebt das High-Byte
  // | kombiniert beide Bytes
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // Temperatur ignorieren

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void setup() {
  Serial.begin(115200);

  // Start I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  setupMPU6050();
}

void loop() {
  getSensorData();

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