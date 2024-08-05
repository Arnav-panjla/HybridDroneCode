#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4

float KpTerm = 0;
float KiTerm = 0;
float KdTerm = 0;

float totalAngle = 0;

volatile float error=0;
volatile float prevError=0;
volatile float errorSum=0;

volatile long prevTime = 0;
volatile long currTime = 0;

const float targetAngle = 0;

const float Kp  = 8.5;
const float Kd = 0.6;
const float Ki =  0;

float PIDterm = 0;

#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

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


  Serial.begin(115200);

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();

  delay(3000);
  for (int i = 0; i < 50; i++) {
    sensors_event_t event;
    bno.getEvent(&event);

    totalAngle += (float)event.orientation.y;
  }
  totalAngle = totalAngle/50;
}

void loop () {

  sensors_event_t event;
  bno.getEvent(&event);

  error = (float)event.orientation.y-totalAngle;
  currTime = millis();

  KpTerm = error*Kp;
  KdTerm = Kd*(error-prevError)/(currTime-prevTime+1);

  PIDterm = KpTerm + KdTerm;

  Serial.print(error);
  Serial.print(".....");
  Serial.print("KpTerm: ");
  Serial.print(KpTerm);
  Serial.print("   KdTerm:    ");
  Serial.println(KdTerm);

  setMotors(-(int)PIDterm, -(int)PIDterm);



  prevError = error;
  prevTime = currTime;
 
}
