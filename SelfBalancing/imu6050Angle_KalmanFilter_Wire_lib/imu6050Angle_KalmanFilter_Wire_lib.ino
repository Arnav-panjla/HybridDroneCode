# include <Wire.h>

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;
float RateRoll, RatePitch, RateYaw;

uint32_t LoopTimer;

// Kalman values
float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 2 * 2;
float KalmanFilterOutput[] = {0, 0};

// Variables for calibration
float RateCalibrationRoll = 0;
float RateCalibrationPitch = 0;
float RateCalibrationYaw = 0;
int RateCalibrationNumber = 0;


const float PIE = 3.14159;
// values to be added
const float AccXee = 0;
const float AccYerr = 0;
const float AccZerr = 0;




void gyro_signals(void){
  //switch on low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10); // ASF_SEL range 2 -> 8g
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read()<<8 | Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | Wire.read();
  // convert acclerometer measurments
  AccX = (float)AccXLSB/4096;
  AccY = (float)AccYLSB/4096;
  AccZ = (float)AccZLSB/4096;

  //converting accleration values to angles
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))/(PIE/180);
  AnglePitch = atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))/(PIE/180);


  // gyro output and pull rotaion rate measurments
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();
  // converting gyro measurments
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;
  
}

void kalmanFilter(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  KalmanFilterOutput[0] = KalmanState;
  KalmanFilterOutput[1] = KalmanUncertainty;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  Wire.setClock(4000000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;

}



void loop() {
  // put your main code here, to run repeatedly:
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalmanFilter(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=KalmanFilterOutput[0]; 
  KalmanUncertaintyAngleRoll=KalmanFilterOutput[1];
  kalmanFilter(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=KalmanFilterOutput[0]; 
  KalmanUncertaintyAnglePitch=KalmanFilterOutput[1];



  Serial.print("AccX = ");
  Serial.print(AccX);
  Serial.print("AccY = ");
  Serial.print(AccY);
  Serial.print("AccZ = ");
  Serial.print(AccZ);
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);
  
  
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();

}


