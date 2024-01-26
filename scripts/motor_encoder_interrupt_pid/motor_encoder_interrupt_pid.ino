#include <PID_v1.h>
// ########## MOTOR VARIABLES ##############
// Motor 1
int PWM1 = 5;
int INA1 = 6;
int INB1 = 4;
// Motor 2
int PWM2 = 9;
int INA2 = 7;
int INB2 = 8;
// Gear and encoder parameters
const float PPR = 16;
const float gearRatio = 90;
const float decodeNumber = 2;
// Initial variables
double rotationalAngleMotor1 = 0.0;
double rotationalAngleMotor2 = 0.0;

// ########## ENCODER VARIABLES ##############
// Pin configuration
const int encoderPinMotor1A = 2;  // Connect encoder channel A Motor 1 to digital pin 2
const int encoderPinMotor1B = 10;  // Connect encoder channel B Motor 1 to digital pin 2
const int encoderPinMotor2A = 3;  // Connect encoder channel A Motor 2 to digital pin 3
const int encoderPinMotor2B = 11;  // Connect encoder channel A Motor 2 to digital pin 3
// Initial variables
volatile long encoderPositionMotor1 = 0;  // Encoder position, volatile as it's modified in an interrupt
volatile long encoderPositionMotor2 = 0;

// ########## PID VARIABLES ##############
// Motor 1
double setpointMotor1 = 0.0;      // Desired position (angle in this case)
double inputMotor1, outputMotor1;
double Kp_M1 = 5.0, Ki_M1 = 0.1, Kd_M1 = 0.0;  // PID tuning parameters
PID pid_M1(&inputMotor1, &outputMotor1, &setpointMotor1, Kp_M1, Ki_M1, Kd_M1, DIRECT);
float current_error_motor1 = 0.0;
// Motor 2
double setpointMotor2 = 0.0;      // Desired position (angle in this case)
double inputMotor2, outputMotor2;
double Kp_M2 = 5.0, Ki_M2 = 0.1, Kd_M2 = 0.0;  // PID tuning parameters
PID pid_M2(&inputMotor2, &outputMotor2, &setpointMotor2, Kp_M2, Ki_M2, Kd_M2, DIRECT);
float current_error_motor2 = 0.0;
//
const float error_umbral = 1.0;

void setup() {
  // ########## ENCODER SETUP ##############
  // Set up the encoder pins as inputs with pullup resistors
  pinMode(encoderPinMotor1A, INPUT_PULLUP);
  pinMode(encoderPinMotor1B, INPUT_PULLUP);
  pinMode(encoderPinMotor2A, INPUT_PULLUP);
  pinMode(encoderPinMotor2B, INPUT_PULLUP);
  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinMotor1A), handleEncoderMotor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinMotor2A), handleEncoderMotor2, CHANGE);

  // ########## MOTOR SETUP ##############
  // Motor 1 initial function
  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  // Motor 2 initial function
  pinMode(PWM2, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);

  // ########## PID SETUP ##############
  pid_M1.SetMode(AUTOMATIC);
  pid_M1.SetOutputLimits(-200, 200);  // Adjust output limits based on your motor driver
  pid_M1.SetSampleTime(10);  // Set PID sample time in milliseconds
  pid_M2.SetMode(AUTOMATIC);
  pid_M2.SetOutputLimits(-200, 200);  // Adjust output limits based on your motor driver
  pid_M2.SetSampleTime(10);  // Set PID sample time in milliseconds

  // ########## ALGORITHM SETUP ##############
  // Other setup code
  Serial.begin(115200);
  
}

void loop() {
  if (Serial.available() > 0){
    String temp_string = Serial.readStringUntil('\n');
    int number = temp_string.toInt();
    setpointMotor1 = number;
  }
  rotationalAngleMotor1 = (360.0 * encoderPositionMotor1) / (PPR * decodeNumber * gearRatio);
  rotationalAngleMotor2 = (360.0 * encoderPositionMotor2) / (PPR * decodeNumber * gearRatio);
  inputMotor1 = rotationalAngleMotor1;
  inputMotor2 = rotationalAngleMotor2;
  current_error_motor1 = setpointMotor1 - rotationalAngleMotor1;
  current_error_motor2 = setpointMotor2 - rotationalAngleMotor2;
  pid_M1.Compute();
  pid_M2.Compute();

  if (outputMotor1 > 0) {
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
  } else {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
  }

  if (outputMotor2 > 0) {
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
  } else {
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
  }

  float temp_output_M1 = outputMotor1;
  if (abs(outputMotor1) < 10){
    temp_output_M1 = 0.0;
  }
  analogWrite(PWM1, abs(temp_output_M1));

  float temp_output_M2 = outputMotor2;
  if (abs(outputMotor2) < 10){
    temp_output_M2 = 0.0;
  }
  analogWrite(PWM2, abs(temp_output_M2));

  // Print the encoder position for demonstration
  Serial.print("Angle position: ");
  Serial.print(rotationalAngleMotor1);
  Serial.print(" | Current error: ");
  Serial.print(current_error_motor1);
  Serial.print(" | Encoder position: ");
  Serial.print(encoderPositionMotor1);
  Serial.print(" | Output od PID: ");
  Serial.println(outputMotor1);

  delay(100);
}

// Interrupt service routine for the encoder
void handleEncoderMotor1() {
  // Read the current state of the two channels
  int stateA = digitalRead(encoderPinMotor1A);
  int stateB = digitalRead(encoderPinMotor1B);

  // Update the encoder position based on the quadrature encoding
  if (stateA == stateB) {
    // Clockwise rotation
    encoderPositionMotor1++;
  } else {
    // Counterclockwise rotation
    encoderPositionMotor1--;
  }
}

void handleEncoderMotor2() {
  // Read the current state of the two channels
  int stateA = digitalRead(encoderPinMotor2A);
  int stateB = digitalRead(encoderPinMotor2B);

  // Update the encoder position based on the quadrature encoding
  if (stateA == stateB) {
    // Clockwise rotation
    encoderPositionMotor2++;
  } else {
    // Counterclockwise rotation
    encoderPositionMotor2--;
  }
}