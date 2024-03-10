// Libraries
#include <PID_v1.h>

// Headers of external functions
void handleEncoderMotor1();                                           // Interruption of Encoder Pin A of Motor 1
void handleEncoderMotor2();                                           // Interruption of Encoder Pin A of Motor 2
void printMotorStates(String, double, float, volatile long, double);  // It prints out the states of the motors

// ################################################
// ########## MOTOR VARIABLES & PINS ##############
// ################################################
// Motor 1 - Orientation
int PWM1 = 5;
#define INA1 A5
#define INB1 A4
// Motor 2 - Orientation
int PWM2 = 6;
#define INA2 A3
#define INB2 A2
// Gear and encoder parameters
const float PPR = 16;
const float gearRatio = 90;
const float decodeNumber = 2;
// Initial variables
double rotationalAngleMotor1 = 0.0;
double rotationalAngleMotor2 = 0.0;

// ##################################################
// ########## ENCODER VARIABLES & PINS ##############
// ##################################################
// Pin configuration
const int encoderPinMotor1A = 2;  // Connect encoder channel A Motor 1 to digital pin 2
const int encoderPinMotor1B = 4;  // Connect encoder channel B Motor 1 to digital pin 4
const int encoderPinMotor2A = 3;  // Connect encoder channel A Motor 2 to digital pin 3
const int encoderPinMotor2B = 7;  // Connect encoder channel A Motor 2 to digital pin 7
// Initial variables
volatile long encoderPositionMotor1 = 0;  // Encoder position, volatile as it's modified in an interrupt
volatile long encoderPositionMotor2 = 0;  // Encoder position, volatile as it's modified in an interrupt

// ###################################################
// ########## PID INSTANCES & VARIABLES ##############
// ###################################################
// Motor 1
double setpointMotor1 = 0.0;  // Desired position (angle in this case)
double inputMotor1, outputMotor1;
double Kp_M1 = 5.0, Ki_M1 = 0.1, Kd_M1 = 0.0;  // PID tuning parameters
PID pid_M1(&inputMotor1, &outputMotor1, &setpointMotor1, Kp_M1, Ki_M1, Kd_M1, DIRECT);
float current_error_motor1 = 0.0;
// Motor 2
double setpointMotor2 = 0.0;  // Desired position (angle in this case)
double inputMotor2, outputMotor2;
double Kp_M2 = 5.0, Ki_M2 = 0.1, Kd_M2 = 0.0;  // PID tuning parameters
PID pid_M2(&inputMotor2, &outputMotor2, &setpointMotor2, Kp_M2, Ki_M2, Kd_M2, DIRECT);
float current_error_motor2 = 0.0;
// Error threshold for negligible error in Stable State
#define ERROR_THRESHOLD 10.0

void setup() {
  // #######################################
  // ########## ENCODER SETUP ##############
  // Set up the encoder pins as inputs with pullup resistors
  pinMode(encoderPinMotor1A, INPUT_PULLUP);
  pinMode(encoderPinMotor1B, INPUT_PULLUP);
  pinMode(encoderPinMotor2A, INPUT_PULLUP);
  pinMode(encoderPinMotor2B, INPUT_PULLUP);
  // Attach interrupts to the encoder pins
  // Arduino R4 Minima
  // 2, 3 pins available for Interruptions
  attachInterrupt(digitalPinToInterrupt(encoderPinMotor1A), handleEncoderMotor1, CHANGE);  // Pin 2
  attachInterrupt(digitalPinToInterrupt(encoderPinMotor2A), handleEncoderMotor2, CHANGE);  // Pin 3

  // #####################################
  // ########## MOTOR SETUP ##############
  // Motor 1 initial setup
  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  // Motor 2 initial setup
  pinMode(PWM2, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);

  // ###################################
  // ########## PID SETUP ##############
  // PID for Motor 1 Orientation
  pid_M1.SetMode(AUTOMATIC);
  pid_M1.SetOutputLimits(-50, 50);  // Adjust output limits based on your motor driver - [0, 255] -> Level of PWM output -> Speed
  pid_M1.SetSampleTime(5);          // Set PID sample time in milliseconds
  // PID for Motor 2 Orientation
  pid_M2.SetMode(AUTOMATIC);
  pid_M2.SetOutputLimits(-50, 50);  // Adjust output limits based on your motor driver - [0, 255] -> Level of PWM output -> Speed
  pid_M2.SetSampleTime(5);          // Set PID sample time in milliseconds

  // #########################################
  // ########## SERIAL SETUP ##############
  Serial.begin(115200);
}

void loop() {
  // Communication for setting another Setpoint
  if (Serial.available() > 0) {
    /*
    Example:
    Input of serial monitor: 
    - 120,26 (Press enter to send "\n")
    Output: 
    - setpointMotor1 = 120
    - setpointMotor2 = 26
    */
    String temp_string = Serial.readStringUntil('\n'); // Read unitl "New line"
    int commaIndex = temp_string.indexOf(','); // Get the index of the comma ","
    if (commaIndex != -1) { // Ensure comma is found
      // Separate numbers
      String var1_string = temp_string.substring(0, commaIndex); // First number
      String var2_string = temp_string.substring(commaIndex + 1); // Second number
      // String to int
      int var1 = var1_string.toInt();
      int var2 = var2_string.toInt(); 
      // Update the setpoins
      setpointMotor1 = var1;
      setpointMotor2 = var2;
    }
  }

  // ############ Interruption explanation ################
  // For every moment that Pin 2 and Pin 3 changes
  // -> handleEncoderMotor1 or handleEncoderMotor2 will run in the background
  // These functions will modify the variables:
  // "encoderPositionMotor1" and "encoderPositionMotor2"
  // ######################################################

  // Calculating the angle in degrees of the motors
  rotationalAngleMotor1 = (360.0 * encoderPositionMotor1) / (PPR * decodeNumber * gearRatio); // Degree
  rotationalAngleMotor2 = (360.0 * encoderPositionMotor2) / (PPR * decodeNumber * gearRatio); // Degree

  // Update the input value of the PID due to pointers.
  // Feedback of the actual position of the motor.
  inputMotor1 = rotationalAngleMotor1; // Degree
  inputMotor2 = rotationalAngleMotor2; // Degree

  // Current error of the motor
  current_error_motor1 = setpointMotor1 - rotationalAngleMotor1;
  current_error_motor2 = setpointMotor2 - rotationalAngleMotor2;

  // Compute the PID to get a new output.
  // New output is updated due to pointers.
  pid_M1.Compute();
  pid_M2.Compute();

  // Clockwise (CW) or Counterclockwise (CCW)
  // !!!!! CHECK IF THE DIRECTION OF THE ROTATION OF THE
  // MOTOR ALIGNS WITH THE OUTPUT OF THE PID IN THE TEST !!!!!
  // Change it LOW instead of HIGH and viceversa
  // Motor1
  if (outputMotor1 > 0) {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
  } else { // outputMotor1 <= 0
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
  }
  // Motor 2
  if (outputMotor2 > 0) {
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
  } else { // outputMotor2 <= 0
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
  }

  // Threshold for negligible error on the position
  // i.e.: Error of 5 degrees -> negligible
  float temp_output_M1 = outputMotor1;
  if (abs(outputMotor1) < ERROR_THRESHOLD) {
    temp_output_M1 = 0.0;
  }
  // Final PWM for motor 1
  analogWrite(PWM1, abs(temp_output_M1));

  float temp_output_M2 = outputMotor2;
  if (abs(outputMotor2) < ERROR_THRESHOLD) {
    temp_output_M2 = 0.0;
  }
  // Final PWM for motor 2
  analogWrite(PWM2, abs(temp_output_M2));

  // Print out the motor and encoder states to supervise
  printMotorStates("Motor 1", rotationalAngleMotor1, current_error_motor1, encoderPositionMotor1, outputMotor1);
  printMotorStates("Motor 2", rotationalAngleMotor2, current_error_motor2, encoderPositionMotor2, outputMotor2);
}

// Interrupt service routine for the encoder Motor 1

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

// Interrupt service routine for the encoder Motor 2

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

void printMotorStates(String motor, double rotationalAngle, float current_error, volatile long encoderPosition, double output) {
  Serial.print(motor);
  Serial.print(" | Angle position: ");
  Serial.print(rotationalAngle);
  Serial.print(" | Current error: ");
  Serial.print(current_error);
  Serial.print(" | Encoder position: ");
  Serial.print(encoderPosition);
  Serial.print(" | Output od PID: ");
  Serial.println(output);
}