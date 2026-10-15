#include <Arduino.h>
#include <Wire.h>

const int IN1 = 26, IN2 = 27;   // left:  PWM on IN1 = forward
const int IN3 = 32, IN4 = 33;   // right: PWM on IN3 = forward
const uint8_t MPU = 0x68;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool readImu(float &ax, float &ay, float &az, float &gz) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();   // high byte arrives first
    uint8_t lo = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  ax = r[0] / 16384.0; ay = r[1] / 16384.0; az = r[2] / 16384.0;  // g
  gz = r[6] / 131.0;                                              // deg/s
  return true;
}

void setup() {
  Serial.begin(115200);
  for (int p : {IN1, IN2, IN3, IN4}) { pinMode(p, OUTPUT); analogWrite(p, 0); }
  Wire.begin(21, 22);  // SDA = G21, SCL = G22 on every bot
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("I2C device at 0x%02X\n", a);
  }
  Serial.println("scan done");
  mpuWrite(0x6B, 0);  // wake the MPU6050
  Wire.beginTransmission(MPU); Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU, (uint8_t)1);
  Serial.printf("WHO_AM_I = 0x%02X (want 0x68)\n", Wire.available() ? Wire.read() : 0);
  Serial.println("keys: l = left fwd, r = right fwd, i = imu 5 s");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'l' || c == 'r') {
    int pin = (c == 'l') ? IN1 : IN3;
    Serial.printf("%s wheel forward 1 s\n", c == 'l' ? "left" : "right");
    analogWrite(pin, 180);
    delay(1000);
    analogWrite(pin, 0);
  } else if (c == 'i') {
    for (int k = 0; k < 25; k++) {
      float ax, ay, az, gz;
      if (readImu(ax, ay, az, gz))
        Serial.printf("ax %+.2f  ay %+.2f  az %+.2f  gz %+7.1f\n", ax, ay, az, gz);
      else Serial.println("imu read failed");
      delay(200);
    }
  }
}
