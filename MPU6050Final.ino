//MPU6050 I2C address 0x68 for first, mpu 0x69 for second
//pins A4 is SDA and A5 is SCL
//using 3.3v for MPU6050

//TO DO:
//implement yaw

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address 0x68 for first, mpu 0x69 for second
float AccX, AccY, AccZ;
float AccXnoise = 0.01;
float AccYnoise = 0.01;
float AccZnoise = 0.05;
float GyroX, GyroY, GyroZ;
float GyroXnoise = 0.1;
float GyroYnoise = 0.1;
float GyroZnoise = 0.1;
float roll, pitch, yaw;
float AccRoll, AccPitch;
float elapsedTime, currentTime, previousTime;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float KalmanAngleYaw = 0, KalmanUncertaintyAngleYaw = 4;
float Kalman1DOutput[] = {0, 0};
uint32_t LoopTimer;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {//KalmanInput - rotation rate, KalmanMeasurement - accelerometer angle, KalmanState - past Kalman filter
  KalmanState += 0.004 * KalmanInput;//gyro contribution to next state
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  
  float KalmanGain = 1;
//  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);//adding in change from accelerometer
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();                      // Initialize comunication
  delay(250);
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission();        //end the transmission

  delay(10);
}
void loop() {
  Read();

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  //adust acceleration and gyro readings for noise
  //using only high values from gyroscope because accelerometer accounts for small movements
  roll += (abs(GyroY) < GyroYnoise ? 0 : GyroY) * elapsedTime;
  pitch += (abs(GyroX) < GyroXnoise ? 0 : GyroX) * elapsedTime;
  yaw += (abs(GyroZ) < GyroZnoise ? 0 : GyroY) * elapsedTime;
  
  //rounds to diminish contribution from noise
  AccX = round(AccX / AccXnoise) * AccXnoise;
  AccY = round(AccY / AccYnoise) * AccYnoise;
  AccZ = round(AccZ / AccZnoise) * AccZnoise;

  AccRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ))/ (3.1416 / 180);
  AccPitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ))/ (3.1416 / 180); 

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, roll, AccRoll);//kalman for Roll
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, pitch, AccPitch);//kalman for pitch
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];


  // Print the values on the serial monitor
  Serial.print("0: ");
  Serial.print(0);

  Serial.print(" Roll_Angle_[°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch_Angle_[°] ");
  Serial.println(KalmanAnglePitch);

//  Serial.print(" AccX: ");
//  Serial.print(AccX);
//  Serial.print(" AccY: ");
//  Serial.print(AccY);
//  Serial.print(" AccZ: ");
//  Serial.print(AccZ);
//  Serial.print(" GyroX: ");
//  Serial.print(GyroX);
//  Serial.print(" GyroY: ");
//  Serial.print(GyroY);
//  Serial.print(" GyroZ: ");
//  Serial.print(GyroZ);

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
float TwosComp(short bin) {
  if (1 == bin >> 15)
  {
    return ~bin + 1;
  }
  return bin;
}
void Read(void) {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = TwosComp(Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = TwosComp(Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = TwosComp(Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value

  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.beginTransmission(MPU);
  Wire.write(0x43);// Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = TwosComp(Wire.read() << 8 | Wire.read()) / 65.5; // For a 1000deg/s range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = TwosComp(Wire.read() << 8 | Wire.read()) / 65.5;
  GyroZ = TwosComp(Wire.read() << 8 | Wire.read()) / 65.5;
}
