// dont include 9 and 10 as it is used for TCCR1A register 
#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4

int motorPower = 0;

const float LeftSpeedFactor = 1.225;//to be kept between 0-1
const float RightSpeedFactor = 1;//to be kept between 0-1

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, (-1)*leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin, (-1)*rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  motorPower = 120;
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower*LeftSpeedFactor, motorPower*RightSpeedFactor);

}
