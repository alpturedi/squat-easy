#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

const int flexPin = A0;

enum MotorPin {
  SIDE_PIN = 11,
  FRONT_PIN = 10,
  FLEX_PIN = 9
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
  pinMode(FLEX_PIN, OUTPUT);
}

void loop() {
  showSensorData();
  delay(1000);
}

void vibrate(MotorPin motorPin, bool isOn = true) {
  if (isOn) {
    analogWrite(motorPin, 255);
  } else {
    analogWrite(motorPin, 0);
  }
}

void showSensorData() {
  //Flex read
  int flex = -1;
  flex = analogRead(flexPin);

  //  /* Get a new normalized sensor event */
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);  

  //Calculate roll (rotation around Y) and pitch (rotation around X) and show them
  float roll = atan2(accel.acceleration.x, accel.acceleration.y) * 180.0 / PI;
  float pitch = atan2(-accel.acceleration.z, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.x * accel.acceleration.x)) * 180.0 / PI;

  //yaw (rotation around Z) is much more complicated to calculate in this way
  Serial.println();
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.println("°");
  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.println("°");
  Serial.print("Flex: ");
  Serial.print(flex);
  Serial.println();

  if (abs(pitch) > 15.0) {
    Serial.println("Front ON");
    vibrate(FRONT_PIN);
  } else {
    Serial.println("Front OFF");
    vibrate(FRONT_PIN, false);
  }

  if (abs(roll) > 80.0) {
    Serial.println("Side ON");
    vibrate(SIDE_PIN);
  } else {
    Serial.println("Side OFF");
    vibrate(SIDE_PIN, false);
  }

  if(flex > 700){
    Serial.println("Flex ON");
    vibrate(FLEX_PIN);
  }else{
    Serial.println("Flex OFF");
    vibrate(FLEX_PIN, false);
  }
}