#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

float pitch, roll;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

enum MotorPin {
  SIDE_PIN = 11,
  FRONT_PIN = 10
};

void setup(void) {
  Serial.begin(9600);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();

  pinMode(SIDE_PIN, OUTPUT);
  pinMode(FRONT_PIN, OUTPUT);
}

void loop() {
  showSensorData();
  delay(1000);
}

void vibrate(MotorPin motorPin, bool isOn = true) {
  if (isOn) {
    // analogWrite(motorPin, 255);
  } else {
    analogWrite(motorPin, 0);
  }
}

void showSensorData() {
  //  /* Get a new normalized sensor event */
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  // Serial.print("Temperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  // Serial.print("Gyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");

  //Calculate roll (rotation around Y) and pitch (rotation around X) and show them
  roll = atan2(accel.acceleration.x, accel.acceleration.y) * 180.0 / PI;
  pitch = atan2(-accel.acceleration.z, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.x * accel.acceleration.x)) * 180.0 / PI;

  //yaw (rotation around Z) is much more complicated to calculate in this way
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.println("°");
  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.println("°");
  Serial.println();

  if (abs(pitch) > 15.0) {
    Serial.println("Front ON");
    vibrate(FRONT_PIN);
  } else {
    Serial.println("Front OFF");
    vibrate(FRONT_PIN, false);
  }

  if (abs(roll) < 75.0) {
    Serial.println("Side ON");
    vibrate(SIDE_PIN);
  } else {
    Serial.println("Side OFF");
    vibrate(SIDE_PIN, false);
  }
}