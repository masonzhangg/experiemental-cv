#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// LiDAR Constants
#define LIDAR_LITE_ADDR 0x62
#define LIDAR_MEASURE_REG 0x00
#define LIDAR_READ_REG 0x8F

Servo myServo;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  myServo.attach(9);
}

void loop() {
  for (int angle = 0; angle <= 180; angle += 10) {
    myServo.write(angle);
    delay(200); 

    int distance = getDistance();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" Distance: ");
    Serial.println(distance);
  }
  delay(1000);
}

int getDistance() {
  Wire.beginTransmission(LIDAR_LITE_ADDR);
  Wire.write(LIDAR_MEASURE_REG);
  Wire.write(0x04);
  Wire.endTransmission();

  delay(20);

  Wire.beginTransmission(LIDAR_LITE_ADDR);
  Wire.write(LIDAR_READ_REG);
  Wire.endTransmission();

  Wire.requestFrom(LIDAR_LITE_ADDR, 2);
  if (Wire.available() == 2) {
    int distance = Wire.read() << 8;
    distance |= Wire.read();
    return distance;
  }
  return -1;
}
