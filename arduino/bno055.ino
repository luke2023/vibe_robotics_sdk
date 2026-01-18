#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  if(!bno.begin())
  {
  /* There was a problem detecting the BNO055 ... check your connections */
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);


  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(
    String(euler.x()) + "," +
    String(euler.y()) + "," +
    String(euler.z()) + "," +
    String(quat.w(), 4) + "," +
    String(quat.x(), 4) + "," +
    String(quat.y(), 4) + "," +
    String(quat.z(), 4) + "," +
    String(system) + "\n"
  );
}
