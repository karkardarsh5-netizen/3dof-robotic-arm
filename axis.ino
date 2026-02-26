#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 140
#define SERVOMAX 520

const int xin = A0;
const int yin = A1;

const float L1 = 12.0f;
const float L2 = 9.0f;
const float Y_AXIS_OFFSET = 0.0f;

const int JOYSTICK_DEADZONE_LOW = 580;
const int JOYSTICK_DEADZONE_HIGH = 680;

const float MIN_REACH = fabs(L1 - L2);
const float MAX_REACH = L1 + L2;

const float POSITION_SPEED_X = 8.0f;   // units/sec at full joystick deflection
const float POSITION_SPEED_Z = 8.0f;   // units/sec at full joystick deflection
const float SERVO_SPEED_DPS = 120.0f;  // degrees/sec

const unsigned long INPUT_UPDATE_MS = 20;
const unsigned long SERVO_UPDATE_MS = 20;

const float BASE_MIN_ANGLE = 0.0f;
const float BASE_MAX_ANGLE = 180.0f;
const float SHOULDER_MIN_ANGLE = 40.0f;
const float SHOULDER_MAX_ANGLE = 180.0f;
const float ELBOW_MIN_ANGLE = 0.0f;
const float ELBOW_MAX_ANGLE = 150.0f;

float currentX = 12.0f;
float currentZ = 8.0f;
float currentBase = 90.0f;

float targetBase = 90.0f;
float targetShoulder = 90.0f;
float targetElbow = 90.0f;

float servoBase = 90.0f;
float servoShoulder = 90.0f;
float servoElbow = 90.0f;

unsigned long lastInputUpdate = 0;
unsigned long lastServoUpdate = 0;

void setangle(int channel, float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  int pulse = SERVOMIN + ((angle / 180.0f) * (SERVOMAX - SERVOMIN));
  pwm.setPWM(channel, 0, pulse);
}

float applyDeadzone(int value) {
  if (value >= JOYSTICK_DEADZONE_LOW && value <= JOYSTICK_DEADZONE_HIGH) {
    return 0.0f;
  }

  if (value < JOYSTICK_DEADZONE_LOW) {
    return (value - JOYSTICK_DEADZONE_LOW) / (float)(JOYSTICK_DEADZONE_LOW - 0);
  }

  return (value - JOYSTICK_DEADZONE_HIGH) / (float)(1023 - JOYSTICK_DEADZONE_HIGH);
}

void clampReach(float &x, float &z) {
  float planarDistance = sqrt((x * x) + (Y_AXIS_OFFSET * Y_AXIS_OFFSET));
  float totalReach = sqrt((z * z) + (planarDistance * planarDistance));

  if (totalReach < 0.0001f) {
    x = MIN_REACH;
    z = 0.0f;
    return;
  }

  float clampedReach = constrain(totalReach, MIN_REACH, MAX_REACH);
  if (clampedReach != totalReach) {
    float scale = clampedReach / totalReach;
    x *= scale;
    z *= scale;
  }
}

float moveToward(float currentValue, float targetValue, float maxStep) {
  float delta = targetValue - currentValue;

  if (fabs(delta) <= maxStep) {
    return targetValue;
  }

  return currentValue + (delta > 0.0f ? maxStep : -maxStep);
}

void updateInverseKinematicsTargets() {
  float planarDistance = sqrt((currentX * currentX) + (Y_AXIS_OFFSET * Y_AXIS_OFFSET));
  float armLength = sqrt((currentZ * currentZ) + (planarDistance * planarDistance));
  armLength = constrain(armLength, MIN_REACH, MAX_REACH);

  float shoulderBase = atan2(currentZ, planarDistance) * 180.0f / PI;

  float shoulderAcosInput = ((sq(L1) + sq(armLength) - sq(L2)) / (2.0f * L1 * armLength));
  shoulderAcosInput = constrain(shoulderAcosInput, -1.0f, 1.0f);
  float shoulderOffset = acos(shoulderAcosInput) * 180.0f / PI;

  float elbowAcosInput = ((sq(L1) + sq(L2) - sq(armLength)) / (2.0f * L1 * L2));
  elbowAcosInput = constrain(elbowAcosInput, -1.0f, 1.0f);
  float elbowAngle = acos(elbowAcosInput) * 180.0f / PI;

  float shoulderAngle = shoulderBase + shoulderOffset;

  targetBase = constrain(currentBase, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  targetShoulder = constrain(shoulderAngle + 45.0f, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
  targetElbow = constrain(180.0f - elbowAngle, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
}

void setup() {
  pwm.begin();
  Serial.begin(9600);
  pwm.setPWMFreq(50);

  pinMode(xin, INPUT);
  pinMode(yin, INPUT);

  clampReach(currentX, currentZ);
  updateInverseKinematicsTargets();

  servoBase = targetBase;
  servoShoulder = targetShoulder;
  servoElbow = targetElbow;

  setangle(15, servoBase);
  setangle(14, servoShoulder);
  setangle(13, servoElbow);

  lastInputUpdate = millis();
  lastServoUpdate = lastInputUpdate;
}

void loop() {
  unsigned long now = millis();

  if (now - lastInputUpdate >= INPUT_UPDATE_MS) {
    float dt = (now - lastInputUpdate) / 1000.0f;
    lastInputUpdate = now;

    int xRead = analogRead(xin);
    int yRead = analogRead(yin);

    float xInput = applyDeadzone(xRead);
    float zInput = applyDeadzone(yRead);

    currentX += xInput * POSITION_SPEED_X * dt;
    currentZ -= zInput * POSITION_SPEED_Z * dt;

    clampReach(currentX, currentZ);
    updateInverseKinematicsTargets();

    Serial.print("b: ");
    Serial.print(targetBase);
    Serial.print(" || s: ");
    Serial.print(targetShoulder);
    Serial.print(" || e: ");
    Serial.print(targetElbow);
    Serial.print(" || x: ");
    Serial.print(currentX);
    Serial.print(" || z: ");
    Serial.print(currentZ);
    Serial.print(" || xval: ");
    Serial.print(xRead);
    Serial.print(" || yval: ");
    Serial.println(yRead);
  }

  if (now - lastServoUpdate >= SERVO_UPDATE_MS) {
    float dt = (now - lastServoUpdate) / 1000.0f;
    lastServoUpdate = now;

    float maxServoStep = SERVO_SPEED_DPS * dt;

    servoBase = moveToward(servoBase, targetBase, maxServoStep);
    servoShoulder = moveToward(servoShoulder, targetShoulder, maxServoStep);
    servoElbow = moveToward(servoElbow, targetElbow, maxServoStep);

    setangle(15, constrain(servoBase, BASE_MIN_ANGLE, BASE_MAX_ANGLE));
    setangle(14, constrain(servoShoulder, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE));
    setangle(13, constrain(servoElbow, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE));
  }
}
