#include <Wire.h>

const int MPU_addr = 0x68;

const int ACCX = 0x3B;
const int ACCY = 0x3D;
const int ACCZ = 0x3F;
const int GYRX = 0x43;
const int GYRY = 0x45;
const int GYRZ = 0x47;

class MPU6050 {
public:
  void readData();
  int16_t accelX() {
    return data[0];
  };
  int16_t accelY() {
    return data[1];
  };
  int16_t accelZ() {
    return data[2];
  };
private:
  int16_t data[3];
};

void MPU6050::readData() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(ACCX);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 6);
  for (int n = 0; n < 6; n++) {
    data[n] = Wire.read() << 8 | Wire.read();
  }
}

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // reset Board
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0b00000001);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6C);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyroscope config
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  // +- 250 deg/s
  Wire.write(0b00000000);
  Wire.endTransmission();

  // Accelerometer config
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  // +- 2g
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  // mpu.readData();
  // Serial.write(127);
  // Serial.write(mpu.accelX());
  // Serial.write(mpu.accelY());
  // Serial.write(mpu.accelZ());
  Serial.write(0);
  Wire.beginTransmission(MPU_addr);
  Wire.write(ACCX);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 6);
  for (int n = 0; n < 6; n++) {
    Serial.write(Wire.read());
  }
  delay(100);
}
