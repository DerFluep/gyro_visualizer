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

int16_t accX_off = 0;
int16_t accY_off = 0;
int16_t accZ_off = 0;
int16_t gyrX_off = 0;
int16_t gyrY_off = 0;
int16_t gyrZ_off = 0;

int16_t read16bit(int address) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 2);
  return Wire.read() << 8 | Wire.read();
}

void calibrate() {
  long sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  long sumGyrX = 0, sumGyrY = 0, sumGyrZ = 0;

  for (int i = 0; i < 200; i++) {
    int16_t rawAccX = read16bit(ACCX);
    int16_t rawAccY = read16bit(ACCY);
    int16_t rawAccZ = read16bit(ACCZ);

    int16_t rawGyrX = read16bit(GYRX);
    int16_t rawGyrY = read16bit(GYRY);
    int16_t rawGyrZ = read16bit(GYRZ);

    sumAccX += rawAccX;
    sumAccY += rawAccY;
    sumAccZ += rawAccZ;

    sumGyrX += rawGyrX;
    sumGyrY += rawGyrY;
    sumGyrZ += rawGyrZ;
  }

  accX_off = sumAccX / 200;
  accY_off = sumAccY / 200;
  accZ_off = (sumAccZ / 200) - 16384;

  gyrX_off = sumGyrX / 200;
  gyrY_off = sumGyrY / 200;
  gyrZ_off = sumGyrZ / 200;

  // Serial.print("AccX off: ");
  // Serial.print(accX_off);
  // Serial.print(" | ");
  // Serial.print("AccY off: ");
  // Serial.print(accY_off);
  // Serial.print(" | ");
  // Serial.print("AccZ off: ");
  // Serial.println(accZ_off);
  //
  // Serial.print("GyrX off: ");
  // Serial.print(gyrX_off);
  // Serial.print(" | ");
  // Serial.print("GyrY off: ");
  // Serial.print(gyrY_off);
  // Serial.print(" | ");
  // Serial.print("GyrZ off: ");
  // Serial.println(gyrZ_off);
  delay(1000);
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

  // calibrate();
}

void loop() {
  mpu.readData();
  Serial.println(mpu.accelY() / 16384.0);
  delay(100);
  // int16_t raw_accx = read16bit(ACCX) - accX_off;
  // float accx = (float)raw_accx / 16384.0;
  // int16_t raw_accy = read16bit(ACCY) - accY_off;
  // float accy = (float)raw_accy / 16384.0;
  // int16_t raw_accz = read16bit(ACCZ) - accZ_off;
  // float accz = (float)raw_accz / 16384.0;
  // Serial.print(accx);
  // Serial.print(" | ");
  // Serial.print(accy);
  // Serial.print(" | ");
  // Serial.println(accz);
  // delay(250);
}
