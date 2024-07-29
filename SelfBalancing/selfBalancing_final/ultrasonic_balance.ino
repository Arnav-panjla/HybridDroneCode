#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4

const int trigR = 9;
const int echoR = 10;
const int trigL = 11;
const int echoL = 12;

float KpTerm = 0;
float KiTerm = 0;
float KdTerm = 0;

volatile float error=0;
volatile float prevError=0;
volatile float errorSum=0;

const float targetAngle = 0;

const float Kp  = 13;
const float Kd = 0.5;
const float Ki =  0;

double distanceR;
double distanceL;

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

void setup() {
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);

  pinMode(trigR, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoR, INPUT); // Sets the echoPin as an Input

  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);

  Serial.begin(115200);
}

void loop () {
  digitalWrite(trigR, LOW);
  digitalWrite(trigL, LOW);
  delayMicroseconds(2);

  digitalWrite(trigL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);
  distanceL = pulseIn(echoL, HIGH)*0.034/2;

  digitalWrite(trigR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigR, LOW);
  distanceR = pulseIn(echoR, HIGH)*0.034/2;

  Serial.print(distanceR);
  Serial.print(" ");
  Serial.println(distanceL);

  if (abs(distanceR-distanceL)>0.2) {
    if (distanceR > distanceL) {
      setMotors(-50, -50);
    } else {
      setMotors(50,50);
    }
    
  } else {
    setMotors(0,0);
  }
}
