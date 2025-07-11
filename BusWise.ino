#include <Wire.h>
#include <math.h>
#include<SoftwareSerial.h>
#include <MPU6050.h>

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer; 
float VelocityX;
float VelocityY;
float Velocity_bus;
float KalmanAngleRoll = 0, KalmanUn_AngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUn_AnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

//Threshholds for detecting an accident 
const float RollThreshould = 15.0;
const float PitchThreshould = 10.0;
const float VelocityLim = 80.0;

SoftwareSerial SIM900A(7,8);

#define RX_PIN 10
#define TX_PIN 11
SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

void Kalman_1d(float &KalmanState, float &KalmanUn, float kalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * kalmanInput; // Corrected KalmanInput
  KalmanUn = KalmanUn + 0.004 * 4 * 4;
  float KalmanGain = KalmanUn * 1 / (1 * KalmanUn + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUn = (1 - KalmanGain);

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUn;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  int16_t TempLSB = Wire.read() << 8 | Wire.read();
  int16_t GyroXLSB = Wire.read() << 8 | Wire.read();
  int16_t GyroYLSB = Wire.read() << 8 | Wire.read();
  int16_t GyroZLSB = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroXLSB / 65.5;
  RatePitch = (float)GyroYLSB / 65.5;
  RateYaw = (float)GyroZLSB / 65.5;

  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.11;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

    MPU6050 accelgyro;
  accelgyro.initialize();

  Serial.begin(57600);
  SIM900A.begin(57600);

  Serial.begin(57600);
  gpsSerial.begin(57600);
  
}

void loop() {
  gyro_signals();
  Kalman_1d(KalmanAngleRoll, KalmanUn_AngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUn_AngleRoll = Kalman1DOutput[1];
  Kalman_1d(KalmanAnglePitch, KalmanUn_AnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUn_AnglePitch = Kalman1DOutput[1];

//calculating acceleration in x and y directions
 
  float AccXInertial = cos(AnglePitch * (3.142 / 180)) * AccX - sin(AnglePitch * (3.142 / 180)) * sin(AngleRoll * (3.142 / 180)) * AccY + cos(AngleRoll * (3.142 / 180)) * AccZ;
  float AccYInertial = cos(AnglePitch * (3.142/ 180)) * AccY + sin(AngleRoll * (3.142 / 180)) * AccZ;

  AccXInertial = (AccXInertial - 1)*9.81;
  AccYInertial = (AccYInertial - 1)*9.81;

  VelocityX = VelocityX + AccXInertial*0.004;
  VelocityY = VelocityY + AccYInertial*0.004;

  Velocity_bus = sqrt((VelocityX*VelocityX) + (VelocityY*VelocityY));

  Serial.print("Roll Rate: ");
  Serial.println(RateRoll, 2);
  Serial.print("Pitch Rate: ");
  Serial.println(RatePitch, 2);
  Serial.print("Yaw Rate: ");
  Serial.println(RateYaw, 2);
  Serial.print("Acceleration X: ");
  Serial.println(AccX, 2);
  Serial.print("Acceleration Y: ");
  Serial.println(AccY, 2);
  Serial.print("Acceleration Z: ");
  Serial.println(AccZ, 2);
  Serial.print("Roll angle: ");
  Serial.println(KalmanAngleRoll);
  Serial.print("Pitch angle: ");
  Serial.println(KalmanAnglePitch);
  Serial.print("VelocityX [m/s]: ");
  Serial.println(VelocityX);
  Serial.print("VelocityY [m/s]: ");
  Serial.println(VelocityY);
  Serial.print("Velocity of the bus[m/s]: ");
  Serial.println(Velocity_bus); 
  while (micros() - LoopTimer < 4000) {
    
  }
  LoopTimer = micros();
  
  if (abs(KalmanAngleRoll) > RollThreshould || abs(KalmanAnglePitch) > PitchThreshould){
    Serial.println("Accident Detected!!!!!!");
    SIM900A.print("AT+CMGS=\"+94725263276\"\r");
    SIM900A.print("Accident Detected!!!!!");
    SIM900A.println((char)26);
    delay(1000);
  }
  if(Velocity_bus > VelocityLim){
    Serial.println("Speed LImit Exceeded!!!!");
    SIM900A.print("AT+CMGS=\"+94725263276\"\r");
    SIM900A.print("Speed Limit Exceeded!!!!!");
    SIM900A.println((char)26);
    delay(1000);
  }
    if (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.print(c);
  }
  if (Serial.available()) {
    char c = Serial.read();
    gpsSerial.print(c);
  }  
  delay(1000);
}




