#include <Wire.h>
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

#define SDA_PIN 32
#define SCL_PIN 33

const uint8_t MPU9250_ADDR = 0x68;

float accelX, accelY, accelZ;
float angleRollAccl, anglePitchAccl, angleYawAccl;
float accNorm;

float rateRoll, ratePitch, rateYaw;
float angleRollGyro, anglePitchGyro, angleYawGyro;

float angleRollKalman = 0; 
float kalmanUncertaintyRoll = 2*2; // uncertainty of the angle estimation (= 2 degrees)
float anglePitchKalman = 0;
float kalmanUncertaintyPitch = 2*2; // uncertainty of the angle estimation (= 2 degrees)

float KalmanOutput[2] = {0, 0}; // [0] = Gues, [1] = Uncertainty

float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;

float accelX_offset = 0;
float accelY_offset = 0;
float accelZ_offset = 0;

unsigned long lastTime;
float dt;

// --- PARAMETERS FOR THE KALMAN-FILTER ---
// Variablen eigener KF --> anpassen
BLA::Matrix<2, 2> A_KF = {1, -dt, 0,  1};  // Systemmatrix A
BLA::Matrix<2, 1> B_KF = {dt, 0};        // Eingangsmatrix B
BLA::Matrix<1, 2> C_KF = {1, 0};        // Ausgangsmatrix C
// Parameterierung des KF (mehr auf Modell oder Messung verlassen)
BLA::Matrix<2, 2> Q_cov = {0.008*0.008, 0, 0, 0.0002*0.0002};   // Q, Kovarianzmatrix Modellunsicherheit
BLA::Matrix<1, 1> R_cov = {0.01};    // R, Kovarianzmatrix Messrauschen

// Hier keine Einstellungen nötig
BLA::Matrix<2, 2> P_est = Q_cov; // Geschätzte Kovarianzmatrix des Schätzfehlers nach Update
BLA::Matrix<2, 2> P_pred = P_est;// prädizierte Kovarianzmatrix des Schätzfehlers VOR Update
BLA::Matrix<2,1> x_est = {0, 0};
BLA::Matrix<2,1> x_pred = {0, 0};
BLA::Matrix<2, 2> A_KF_T = ~A_KF;  // Systemmatrix A transponiert
BLA::Matrix<2, 1> C_KF_T = ~C_KF;  // Ausgangsmatrix C transponiert
BLA::Matrix<2, 1> K_KF = {0, 0};  // Kalman-Gain-Matrix
BLA::Matrix<2, 2> I_KF = {1, 0, 0, 1};  // 2x2 Einheitsmatrix I

// OUTPUT VARIABLES 

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void setupMPU9250() {
  // MPU9250 aufwecken (Power Management Register)
  // Register 0x6B (PWR_MGMT_1)
  // Wake up (BIT6 can be set to put in sleep mode)
  writeRegister(MPU9250_ADDR, 0x6B, 0x00);

  // Set Low Pass Filter (Register 0x1A)
  // BIT0 * BIT2 decide about Filter (0x03 == 44Hz)
  writeRegister(MPU9250_ADDR, 0x1A, 0x03);

  // Set Sample Rate Divider (Register 0x19)
  // Sample Rate is set by BIT0 * BIT7 (0x04 == 200Hz)
  writeRegister(MPU9250_ADDR, 0x19, 0x04);

  // Set senitivities:
  // Accellerometer: (Register 0x1C)
  // BIT3 * BIT4 decide about sensitivity (0x08 == ±4g)
  writeRegister(MPU9250_ADDR, 0x1C, 0x08);

  // Gyroscope: (Register 0x1B)
  // BIT3 * BIT4 decide about sensitivity (0x08 == ±500 °/s)
  writeRegister(MPU9250_ADDR, 0x1B, 0x08);
  
  //Serial.println("MPU9250 bereit!");
}

void getSensorData() {
  // Begin reading data from register 0x3B
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);  // Block I2C-BUS

  // Request 14 Bytes:
  Wire.requestFrom(MPU9250_ADDR, (size_t)14, true);

  // Get 16 Bit values for each reading
  // << 8 verschiebt das High-Byte
  // | kombiniert beide Bytes
  int16_t rawAccX = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t rawAccY = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t rawAccZ = (int16_t)(Wire.read() << 8 | Wire.read());

  Wire.read(); Wire.read(); // Temperatur ignorieren

  int16_t rawGyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t rawGyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t rawGyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

  // Calculate actual values (depending on sensitivity settings)
  // ±4g -> 8192 LSB/g
  accelX = rawAccX / 8192.0 - accelX_offset;
  accelY = rawAccY / 8192.0 - accelY_offset;
  accelZ = rawAccZ / 8192.0 - accelZ_offset;

  // ±500 °/s -> 65.5 LSB/(°/s)
  rateRoll = rawGyroX / 65.5 - gyroX_offset;
  ratePitch = rawGyroY / 65.5 - gyroY_offset;
  rateYaw = rawGyroZ / 65.5 - gyroZ_offset;
}

void calibrateMPU9250() {
  const int numReadings = 500;
  float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
  float accelX_sum = 0, accelY_sum = 0, accelZ_sum = 0;

  //Serial.println("Calibrating MPU9250...");

  for (int i = 0; i < numReadings; i++) {
    getSensorData();
    gyroX_sum += rateRoll;
    gyroY_sum += ratePitch;
    gyroZ_sum += rateYaw;

    accelX_sum += accelX;
    accelY_sum += accelY;
    accelZ_sum += accelZ;

    delay(5);
  }

  // Calculate offsets:
  gyroX_offset = gyroX_sum / numReadings;
  gyroY_offset = gyroY_sum / numReadings;
  gyroZ_offset = gyroZ_sum / numReadings;

  accelX_offset = accelX_sum / numReadings;
  accelY_offset = accelY_sum / numReadings;
  accelZ_offset = accelZ_sum / numReadings - 1.0; // To account for gravity
}

void calcAngles() {
  // Function to calculate angles from accelerometer data
  // (* 180 / PI) converts from radians to degrees
  angleRollAccl = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 180 / PI;
  anglePitchAccl = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180 / PI;
  accNorm = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);

  // Integrate gyro rates to get angles
  angleRollGyro += rateRoll * dt;
  anglePitchGyro += ratePitch * dt;
  angleYawGyro += rateYaw * dt;

  if (angleRollGyro > 180) angleRollGyro -= 360;
  if (angleRollGyro < -180) angleRollGyro += 360;

  if (abs(accNorm - 1.0) > 0.1) {
    R_cov = {0.8};   // vertraue Acc NICHT
    } 
    
  else {
        R_cov = {0.03};  // normal
    }
}

void calcAnglesKalman() {
  // Updating Systemmatrix with current dt
  A_KF = {1, -dt,
        0,  1};

  B_KF = {dt,
        0};

  A_KF_T = ~A_KF;

  // --- Eingang: Gyro-Rate (°/s) ---
  BLA::Matrix<1,1> u = {rateRoll};

  // --- Messwert ---
  BLA::Matrix<1,1> y = {angleRollAccl};

  // --- Prädiktion ---
  x_pred = A_KF * x_est + B_KF * u;
  P_pred = A_KF * P_est * A_KF_T + Q_cov;

  // --- Kalman Gain ---
  BLA::Matrix<1,1> S = C_KF * P_pred * C_KF_T + R_cov;
  K_KF = P_pred * C_KF_T * Inverse(S);

  // --- Update ---
  x_est = x_pred + K_KF * (y - C_KF * x_pred);
  P_est = (I_KF - K_KF * C_KF) * P_pred;

  // --- Output ---
  angleRollKalman = x_est(0);
}

void setup() {
  Serial.begin(115200);

  // Start I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  setupMPU9250();
  delay(1000); // Warte kurz, damit sich die Sensorwerte stabilisieren können
  calibrateMPU9250();

  getSensorData();

  angleRollAccl = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 57.3;

  x_est(0) = angleRollAccl;  // Startwinkel setzen!
  x_est(1) = 0;              // Bias initial
}

void loop() {
  // Get time difference (dt) for integration of gyro rates
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0; // Time in seconds
  lastTime = micros();

  // --- Systemmatrix mit aktuellem dt updaten ---
  A_KF = {1, -dt,
          0,  1};

  B_KF = {dt,
          0};

  A_KF_T = ~A_KF;

  getSensorData();
  calcAngles();

  // Calculate angles using Kalman Filter:
  calcAnglesKalman();

  //calcAnglesKalman();
  //anglePitchKalman = KalmanOutput[0];
  //kalmanUncertaintyPitch = KalmanOutput[1];

  Serial.print(">roll:");
  Serial.println(angleRollAccl);
  
  Serial.print(">pitch:");
  Serial.println(anglePitchAccl);

  Serial.print(">rollgyro:");
  Serial.println(angleRollGyro);
  
  Serial.print(">pitchgyro:");
  Serial.println(anglePitchGyro);

  Serial.print(">rollKalman:");
  Serial.println(angleRollKalman);
  
  Serial.print(">pitchKalman:");
  Serial.println(anglePitchKalman);

  Serial.print(">K0: ");
  Serial.println(K_KF(0));

  delay(10);
}