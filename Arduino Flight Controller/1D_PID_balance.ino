/* This program uses a PID controller to stabilize a quadcoptor on one axis.
A complementary filter is used on the IMU accelerometer and gyro.
Note: This is not tuned for any drone at the moment*/

#include <SPI.h>
#include <Servo.h>

/// ----------------- Contents --------------------- ///
// 1 - Variables
// 2 - Setup
// 3 - Main Loop
// 4 - Functions

/// ----------------- Variables --------------------- ///

String mode = "run"; // "run" or "debug"

// Pins
int motor1Pin = 5;
int motor2Pin = 6;
int IMUPin = 10; // CS = 10, MOSI/SDO = 11, MISO/SDI = 12, CLK = 13

// PWM values
int minPWM = 1000;
int maxPWM = 2000;
int hoverThrottle = 1257; // Actual hover throttle if motor datasheet is correct
int throttleLimit = 170;
int throttleMin = hoverThrottle - throttleLimit;
int throttleMax = hoverThrottle + throttleLimit;

// IMU Addresses
uint8_t yAccAddress = 0x3D;
uint8_t xGyroAddress = 0x43;
uint8_t gyroConfigAddress = 0x1B;

// Roll angle complementary filter
float rollAngle = 0;
float gyroGain = 0.98;
float accGain = 0.02;

// Timing
float loopFreq = 500; // Hz
float timeStepS = 1/loopFreq;
float timeStepMs = timeStepS*1000;
unsigned long startTime;
unsigned long nextTime;
unsigned long testTime;
unsigned long motorStartupTime = 400; // ms, wait to let sensor readings stabilize, motors startup, and LPFs start
int highCalibrationTime;
int lowCalibrationTime;

// Reference Values
float rollRef = 0; // radians

// PID gains
float Pgain = 41; // ms/rad
float Igain = 5; // ms/(rad-s)
float Dgain = 23; // ms/(rad/s) 20 - 25

// Other PID variables
float rollIntegralLimit = 50; // PWM value
float rollErrorIntegral = 0;
float lastRollError = rollRef;
float rollDerivLPFcutoffFreq = 70; // Hz
float rollDerivSum = 0;
float lastRollDeriv = 0;
float lastRollAngle = 0;

// Etc.
Servo motor1;
Servo motor2;
uint16_t gAcc = 16384; // acceleration value read at 1g
struct sensor {int16_t yAcc; int16_t xGyro;};
float pi = 3.14159265;

/// ----------------- Setup --------------------- ///
void setup() {
  setupTestSchedule();
  setupIMU();
  setupMotors();
  if (mode == "debug") {
    Serial.begin(115200);
  }
  startTime = millis();
  nextTime = startTime;
}

/// ----------------- Main Loop --------------------- ///
void loop() {
  bool testEnded = millis() > startTime + testTime;
  bool frameChange = millis() - nextTime >= timeStepMs;
  if (testEnded) {
    motor1.writeMicroseconds(minPWM);
    motor2.writeMicroseconds(minPWM);
    return;
  }
  if (frameChange) {
    nextTime += timeStepMs;
    auto [yAcc, xGyro] = getSensorMeasurements();
    float rollAngle = calculateOrientation(yAcc, xGyro);
    handleControls(rollAngle);
  }
}

/// ----------------- Functions --------------------- ///

// Set test schedule and duration depending on whether we're in run or debug mode
void setupTestSchedule() {
  if (mode == "run") {
    testTime = 7000; // ms, including startup time
    highCalibrationTime = 5000; // ms
    lowCalibrationTime = 5000; // ms
  } else if (mode == "debug") {
    testTime = 50000;
    highCalibrationTime = 100;
    lowCalibrationTime = 100;
  }
}

// Start SPI communication
void setupIMU() {
  pinMode(IMUPin, OUTPUT);
  SPI.begin();
}

// Setup motor pins as servos (50Hz PWM)
void setupMotors() {
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  calibrateESCs();
}

// Sends a max throttle signal then a min throttle signal for a specified duration to calibrate the ESCs
void calibrateESCs() {
  motor1.writeMicroseconds(maxPWM);
  motor2.writeMicroseconds(maxPWM);
  delay(highCalibrationTime);
  motor1.writeMicroseconds(minPWM);
  motor2.writeMicroseconds(minPWM);
  delay(lowCalibrationTime);
}

// get accelerometer and gyroscope measurements from IMU
sensor getSensorMeasurements() {
  int16_t yAcc = readDataIMU(yAccAddress);
  int16_t xGyro = readDataIMU(xGyroAddress);  
  return sensor {yAcc, xGyro};
}

// Returns 2 bytes of data from the IMU at the specified address
int16_t readDataIMU(uint8_t address) {
  digitalWrite(IMUPin, LOW);
  SPI.transfer(address | 0x80);
  uint8_t highByte = SPI.transfer(0x00);
  uint8_t lowByte = SPI.transfer(0x00);
  digitalWrite(IMUPin, HIGH);
  return (highByte << 8) | lowByte;
}

// Limits the input value to lower and upper saturation values
float saturation(float value, float min, float max) {
  if (value > max) { value = max; };
  if (value < min) { value = min; };
  return value;
}

// Converts raw IMU data to roll angle using a complementary filter
float calculateOrientation(int16_t yAcc, int16_t xGyro) {

  //Accelerometer
  float yAccRatio = saturation((float)yAcc/(float)gAcc, -1, 1);
  float rollAngleAcc = asin(yAccRatio);

  //Gyro
  float xGyroRad = xGyro*250/180*pi/32768;
  float rollAngleGyro = rollAngle + xGyroRad*timeStepS;

  //Sensor fusion
  rollAngle = gyroGain*rollAngleGyro + accGain*rollAngleAcc;

  if (mode == "debug") {
    Serial.print(.7);
    Serial.print(",");
    Serial.print(-.7);
    Serial.print(",");
    Serial.println(rollAngle);
  }
  return rollAngle;
}

// Takes in error value, and outputs sum of P,I, and D components
float calculatePID(float rollError) {

  // Integral
  rollErrorIntegral += rollError*timeStepS;
  rollErrorIntegral = saturation(rollErrorIntegral, -rollIntegralLimit/Igain, rollIntegralLimit/Igain);

  // Derivative with low pass filter
  float rollErrorDeriv = (rollError - rollDerivSum)*rollDerivLPFcutoffFreq;
  rollDerivSum += rollErrorDeriv*timeStepS;

  // PID sum
  // max rollError = 90 deg = 1.57 or 1.5
  // use lookup table for these
  float Pval = Pgain * rollError; // z80 8 bit signed
  float Ival = Igain * rollErrorIntegral; // z80 int8 (8 bit signed)
  float Dval = Dgain * rollErrorDeriv; // z80 int8

  // z80 16 bit -384 to 384, if divide by 4, rollControlVal is from -96 to 96, which is fine bc this includes P,I,&D
  float rollControlVal = Pval + Ival + Dval; // z80 8 bit, (-43 to 43 so 7 bit, put saturation on roll instead of throttle)
  return rollControlVal;
}

// Motor mixing algorithm
void controlMotors(float roll) {

  // Calculate motor mixing
  const int thrust = hoverThrottle; // z80 = 8 bit
  float motor1Throttle = thrust + roll;
  float motor2Throttle = thrust - roll;
  motor1Throttle = saturation(motor1Throttle, throttleMin, throttleMax); // z80 = 8 bit (+1000)
  motor2Throttle = saturation(motor2Throttle, throttleMin, throttleMax); // z80 = 8 bit (+1000)

  // If motor startup is complete, start balancing the drone
  bool motorStartupComplete = millis() > startTime + motorStartupTime;
  if (motorStartupComplete) {
    motor1.writeMicroseconds((int)motor1Throttle);
    motor2.writeMicroseconds((int)motor2Throttle);
  } else {
    motor1.writeMicroseconds(hoverThrottle);
    motor2.writeMicroseconds(hoverThrottle);
    rollErrorIntegral = 0; // so it doesn't windup
  }
}

// Controller
void handleControls(float rollAngle) {
  float rollControlVal = calculatePID(rollRef - rollAngle);
  controlMotors(rollControlVal);
}