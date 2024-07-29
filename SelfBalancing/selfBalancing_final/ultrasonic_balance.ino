const int trigR = 9;
const int echoR = 10;
const int trigL = 11;
const int echoL = 12;
// defines variables
double distanceR;
double distanceL;

void setup() {
  pinMode(trigR, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoR, INPUT); // Sets the echoPin as an Input

  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  // Clears the trigPin
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
}
