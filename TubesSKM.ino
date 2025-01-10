// Right motor
int enableLeftMotor = 5;
int leftMotorPin1 = 7;
int leftMotorPin2 = 8;

// Left motor
int enableRightMotor = 6;
int rightMotorPin1 = 9;
int rightMotorPin2 = 10;

// IR Sensors
int sensorLeftPin = A0;
int sensorMiddleLeftPin = A1;
int sensorCenterPin = A2;
int sensorMiddleRightPin = A3;
int sensorRightPin = A4;

// Threshold sensor IR
int irThreshold = 500;

// PID constants
float kp = 3;
float ki = 4;
float kd = 0.5625;

// PID variables
float lastError = 0;
float integral = 0;

// Define light turn threshold
float lightTurnThreshold = 0.5; // Adjust this threshold as needed

float deadZone = 0.1;

// Base speed for motors
int baseSpeed = 200;

void setup() {
  // Initialize motor pins
  pinMode(enableRightMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Initialize sensor pins
  pinMode(sensorLeftPin, INPUT);
  pinMode(sensorMiddleLeftPin, INPUT);
  pinMode(sensorCenterPin, INPUT);
  pinMode(sensorMiddleRightPin, INPUT);
  pinMode(sensorRightPin, INPUT);

  // Start Serial Monitor for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensor values
  int sensorValues[] = {
    analogRead(sensorLeftPin),
    analogRead(sensorMiddleLeftPin),
    analogRead(sensorCenterPin),
    analogRead(sensorMiddleRightPin),
    analogRead(sensorRightPin)
  };

  // Calculate weighted error
  int weights[] = {-2, -1, 0, 1, 2};
  int sumWeights = 0;
  int sumSensorValues = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] > irThreshold) {
      sumWeights += weights[i] * sensorValues[i];
      sumSensorValues += sensorValues[i];
    }
  }

  float error = 0;
  if (sumSensorValues != 0) {
    error = (float)sumWeights / sumSensorValues;
  }

  if (fabs(error) < deadZone) {
    error = 0;
  }

  // Calculate PID terms
  float derivative = error - lastError;
  integral += error;  // Tambahkan ini untuk mengakumulasi error
  
  // Opsional: tambahkan anti-windup untuk membatasi integral
  integral = constrain(integral, -100, 100);  // Batasi nilai integral agar tidak terlalu besar
      
  float correction = (kp * error) + (ki * integral) + (kd * derivative);

  // Adjust motor speeds based on error
  int adjustedSpeedLeft = baseSpeed + (correction * fabs(error));
  int adjustedSpeedRight = baseSpeed - (correction * fabs(error));

  // Ensure speeds are within valid range
  adjustedSpeedLeft = constrain(adjustedSpeedLeft, 0, 255);
  adjustedSpeedRight = constrain(adjustedSpeedRight, 0, 255); 

  // Apply interpolation for smoother turns
  if (error > 0) {
    // Turning right
    moveMotors(LOW, HIGH, LOW, HIGH, adjustedSpeedLeft, adjustedSpeedRight);
  } else if (error < 0) {
    // Turning left
    moveMotors(LOW, HIGH, LOW, HIGH, adjustedSpeedLeft, adjustedSpeedRight);
  } else {
    // Moving straight
    moveMotors(LOW, HIGH, LOW, HIGH, baseSpeed, baseSpeed);
  }

  lastError = error;

  if (millis() % 500 == 0) {
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print("\tCorrection: ");
    Serial.println(correction);
  }

  Serial.print("Error: ");
  Serial.println(error);
  Serial.println("\tCorrection: ");
  Serial.println(correction);
  Serial.println("\tLeft: ");
  Serial.println(adjustedSpeedLeft);
  Serial.println("\tRight: ");
  Serial.println(adjustedSpeedRight);

  delay(100);
}

// Function to control motor movement with speed parameters
void moveMotors(int leftMotorBackward, int leftMotorForward, int rightMotorBackward, int rightMotorForward, int speedLeft, int speedRight) {
  // Control right motor
  digitalWrite(leftMotorPin1, leftMotorForward);
  digitalWrite(leftMotorPin2, leftMotorBackward);

  // Control left motor
  digitalWrite(rightMotorPin1, rightMotorForward);
  digitalWrite(rightMotorPin2, rightMotorBackward);

  // Set motor speed
  analogWrite(enableRightMotor, speedRight);
  analogWrite(enableLeftMotor, speedLeft);
}