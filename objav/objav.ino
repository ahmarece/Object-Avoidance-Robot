// ----- PID Line Follower (5 IR sensors, no weighted method) -----

// Sensor pins
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4

// Motor pins (example: L298N driver)
#define ENA 9
#define IN1 8
#define IN2 7
#define ENB 10
#define IN3 12
#define IN4 11

// PID constants (tune these)
float Kp = 25;
float Ki = 0;
float Kd = 15;

// Variables
int sensor[5];
int error = 0;
int lastError = 0;
float P, I, D, PID_value;
int motorSpeed = 150; // base speed

void setup() {
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  readSensors();
  calculatePID();
  motorControl();
}

void readSensors() {
  sensor[0] = digitalRead(S1);
  sensor[1] = digitalRead(S2);
  sensor[2] = digitalRead(S3);
  sensor[3] = digitalRead(S4);
  sensor[4] = digitalRead(S5);

  // Define error values (you can adjust based on line thickness/robot response)
  if (sensor[2] == 1) error = 0;        // Center
  else if (sensor[1] == 1) error = -1;  // Slight left
  else if (sensor[0] == 1) error = -2;  // Sharp left
  else if (sensor[3] == 1) error = 1;   // Slight right
  else if (sensor[4] == 1) error = 2;   // Sharp right
  else error = lastError; // If no sensor detects line, keep last error
}

void calculatePID() {
  P = error;
  I += error;
  D = error - lastError;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;
}

void motorControl() {
  int leftMotorSpeed = motorSpeed - PID_value;
  int rightMotorSpeed = motorSpeed + PID_value;

  // limit speed between 0–255
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

