#include "Wire.h"
#include "math.h"


// dont include 9 and 10 as it is used for TCCR1A register 
#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4


float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;
float RateRoll, RatePitch, RateYaw;

volatile float currentAngle;
volatile float prevAngle=0;
volatile float error;
volatile float prevError=0;
volatile float errorSum=0;


// Kalman filter variables
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
const float targetAngle = 0;
const float sampleTime = 0.005;

int motorPower = 0;

const float AccXerr = -0.09;
const float AccYerr = -0.01;
const float AccZerr = -0.84;

const float Kp  = 3.6;
const float Kd = 0;
const float Ki =  0.4;

const float LeftSpeedFactor = 1;//to be kept between 0-1
const float RightSpeedFactor = 1;//to be kept between 0-1


void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, leftMotorSpeed*(-1));
    digitalWrite(leftMotorDirPin, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin, rightMotorSpeed*(-1));
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

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
  AccX = (float)AccXLSB/4096 + AccXerr;
  AccY = (float)AccYLSB/4096 + AccYerr;
  AccZ = (float)AccZLSB/4096 + AccZerr;

  //converting accleration values to angles
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))/(PIE/180);
  AnglePitch = atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))/(PIE/180);


  // gyro output and pull rotaion rate measurments
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
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
  KalmanState = KalmanState + 0.005 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.005 * 0.005 * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  KalmanFilterOutput[0] = KalmanState;
  KalmanFilterOutput[1] = KalmanUncertainty;
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  // put your setup code here, to run once:
  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  Wire.setClock(400000);
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
  digitalWrite(13,HIGH);
  delay(500);

  init_PID();

}



void loop() {
  // put your main code here, to run repeatedly:
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  



  // Serial.print("AccX = ");
  // Serial.print(AccX);
  // Serial.print("  AccY = ");
  // Serial.print(AccY);
  // Serial.print("  AccZ = ");
  // Serial.print(AccZ);
  Serial.print("  Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print("  Pitch Angle [°] ");
  Serial.print(KalmanAnglePitch);
  Serial.print("     MotorPower --> ");
  Serial.print(motorPower);

  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);

  Serial.print("     MotorPower after constrain --> ");
  Serial.println(motorPower);

}

ISR(TIMER1_COMPA_vect)
{
  kalmanFilter(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=KalmanFilterOutput[0]; 
  KalmanUncertaintyAngleRoll=KalmanFilterOutput[1];
  kalmanFilter(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=KalmanFilterOutput[0]; 
  KalmanUncertaintyAnglePitch=KalmanFilterOutput[1];

  // calculate the angle of inclination
  
  currentAngle = KalmanAngleRoll;
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;

  // // toggle the led on pin13 every second
  // count++; // variable will increment every 5 ms

  // if(count == 200)  { // 5*200 = 1000 ms
  //   count = 0;
  //   digitalWrite(13, !digitalRead(13));
  // }
}
