#include <PID_v1.h>

// ########## MOTOR VARIABLES ##############
int PWM2 = 5;
int INA2 = 6;
int INB2 = 7;

const float PPR = 16;
const float gearRatio = 90;
const float decodeNumber = 2;

double rotationalAngle = 0.0;

// ########## ENCODER VARIABLES ##############
// Pin configuration
const int encoderPinA = 2;  // Connect encoder channel A to digital pin 2
const int encoderPinB = 3;  // Connect encoder channel B to digital pin 3
// Variables
volatile long encoderPosition = 0;  // Encoder position, volatile as it's modified in an interrupt

// ########## PID VARIABLES ##############
double setpoint = 0.0;      // Desired position (angle in this case)
double input, output;
double Kp = 10.0, Ki = 0.1, Kd = 0.0;  // PID tuning parameters
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
const float error_umbral = 1.0;
float current_error = 0.0;
// ########## ALGORITHM VARIABLES ##############
//float current_time;
//int flag = 1;

void setup() {
  // ########## ENCODER SETUP ##############
  // Set up the encoder pins as inputs with pullup resistors
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncoder, CHANGE);

  // ########## MOTOR SETUP ##############
  // Motors initial function
  pinMode(PWM2, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);

  // ########## PID SETUP ##############
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-100, 100);  // Adjust output limits based on your motor driver
  pid.SetSampleTime(10);  // Set PID sample time in milliseconds

  // ########## ALGORITHM SETUP ##############
  // Other setup code
  Serial.begin(115200);
  //current_time = millis();
  
}

void loop() {
  if (Serial.available() > 0){
    String temp_string = Serial.readStringUntil('\n');
    int number = temp_string.toInt();
    setpoint = number;
  }
  rotationalAngle = (360.0 * encoderPosition) / (PPR * decodeNumber * gearRatio);
  input = rotationalAngle;
  current_error = setpoint - rotationalAngle;
  pid.Compute();

  if (output > 0) {
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
  } else {
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
  }

  float temp_output = output;
  if (abs(output) < 10){
    temp_output = 0.0;
  }
  analogWrite(PWM2, abs(temp_output));

  // Print the encoder position for demonstration
  Serial.print("Angle position: ");
  Serial.print(rotationalAngle);
  Serial.print(" | Current error: ");
  Serial.print(current_error);
  Serial.print(" | Encoder position: ");
  Serial.print(encoderPosition);
  Serial.print(" | Output od PID: ");
  Serial.println(output);

  delay(100);
}

// Interrupt service routine for the encoder
void handleEncoder() {
  // Read the current state of the two channels
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  // Update the encoder position based on the quadrature encoding
  if (stateA == stateB) {
    // Clockwise rotation
    encoderPosition++;
  } else {
    // Counterclockwise rotation
    encoderPosition--;
  }
}