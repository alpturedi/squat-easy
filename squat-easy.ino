#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

const int flexPin = A0;
const bool IS_LEFT_KNEE = true;

int squatCount = 0;

// int motorValues[3] = {0,0,0};
int motorPins[3] = { 11, 10, 9 };

enum Motor {
  SIDE_PIN = 0,
  FRONT_PIN = 1,
  FLEX_PIN = 2
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

  pinMode(motorPins[SIDE_PIN], OUTPUT);
  pinMode(motorPins[FRONT_PIN], OUTPUT);
  pinMode(motorPins[FLEX_PIN], OUTPUT);
}

void turnOn(Motor motor, bool isOn = true) {
  if (isOn) {
    analogWrite(motorPins[motor], 255);
  } else {
    analogWrite(motorPins[motor], 0);
  }
}

void loop() {
  showSensorData();
  delay(860);
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
  float roll = 90 + (atan2(accel.acceleration.x, accel.acceleration.y) * 180.0 / PI);

  if (IS_LEFT_KNEE) {
    roll *= -1;
  }

  float pitch = atan2(-accel.acceleration.z, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.x * accel.acceleration.x)) * 180.0 / PI;

  //yaw (rotation around Z) is much more complicated to calculate in this way
  Serial.println();
  Serial.print("Roll to Outside: ");
  Serial.print(roll, 1);
  Serial.println("°");
  Serial.print("Prone to Front: ");
  Serial.print(pitch, 1);
  Serial.println("°");
  Serial.print("Flex on Lumbar: ");
  Serial.print(flex);
  Serial.println();

  if (pitch > 25.0) {
    Serial.println("WEIGHT TO YOUR HEELS !");
    turnOn(FRONT_PIN);
  } else {
    turnOn(FRONT_PIN, false);
  }

  if (pitch > 10.0 && roll < 10.0) {
    Serial.println("KNEES OUT !");
    turnOn(SIDE_PIN);
  } else {
    turnOn(SIDE_PIN, false);
  }

  if (flex > 700) {
    Serial.println("LIFT YOUR CHEST UP !");
    turnOn(FLEX_PIN);
  } else {
    turnOn(FLEX_PIN, false);
  }
}