#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

float pitch, roll;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

const int hipRotation = 10;
const int tibiaPitch = 11;

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

  pinMode(tibiaPitch, OUTPUT);
}

void vibrate(){
  analogWrite(tibiaPitch, 255); 
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
  // Serial.print("Accel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  // Serial.print("Gyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");

  //Calculate roll (rotation around Y) and pitch (rotation around X) and show them
  roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
  pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
  //yaw (rotation around Z) is much more complicated to calculate in this way
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.println("°");
  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.println("°");
  Serial.println();

if(pitch < 70.0){
  Serial.println("ON");
  analogWrite(tibiaPitch, 255); 
}else{
  Serial.println("OFF");
  analogWrite(tibiaPitch, 0);
}
  
}

void loop() {
  showSensorData();
  delay(1000);
}
