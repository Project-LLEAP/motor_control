#include <SPI.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
// Motor control pins
#define DAC1 25
#define enable1 33
#define direction1 32
 
// Encoder
#define CS_PIN 5
#define AMT22_NOP 0x00
#define AMT22_ZERO 0x70
#define NUM_POSITIONS_PER_REV 16384
 
// Control settings
#define MOTOR_MOVEMENT_TOLERANCE_DEG 0.5f
#define DEADBAND_U 5.0f
#define CONTROL_PERIOD_US 2000UL     // 2 ms = 500 Hz
#define DEBUG_PERIOD_MS 100UL        // print at 10 Hz
#define MAX_TARGET_SEQUENCE 32
 
// State
bool motorEnabled = false;
 
// Control state
float target_pos = 0.0f;
float prev_error = 0.0f;
float integral_error = 0.0f;
float derivative_error = 0.0f;

// Target sequencing
float targetSequence[MAX_TARGET_SEQUENCE];
int targetSequenceCount = 0;
int targetSequenceIndex = 0;
 
// Tunable gains
float kp = 50.0f;
float ki = 5.0f;
float kd = 2.0f;
 
// Tunable max command
int maxSpeedCmd = 255;
int minSpeedCmd = 30;
 
// Anti-windup clamp
float integral_limit = 50.0f;
 
// Serial command buffer
char cmdBuffer[256];
int cmdIndex = 0;
bool cmdOverflow = false;
 
// Timing
uint32_t lastControlTimeUs = 0;
uint32_t lastDebugTimeMs = 0;
 
// Debug values
float debug_current_pos = 0.0f;
float debug_error = 0.0f;
float debug_u = 0.0f;
int debug_vel = 0;
int debug_dir = 0;

// current velocity
volatile int curVel = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
 
  setup_encoder();
  setup_motor_controller();
 
  disableMotor();
  setMotor(0, 0);
 
  delay(100);
 
  // Leave disabled on boot
  // setZeroSPI(CS_PIN);   // only run manually if needed

 
  target_pos = readEncoderPositionDeg();
 
  lastControlTimeUs = micros();
  lastDebugTimeMs = millis();
 
  Serial.println("Controller ready.");
  Serial.println("Motor is DISABLED.");
  Serial.println("Commands:");
  Serial.println("  E 1   -> enable motor");
  Serial.println("  E 0   -> disable motor");
  Serial.println("  T 45 90 15 -> set one or more target angles");
  Serial.println("  P 5   -> set kp");
  Serial.println("  I 0   -> set ki");
  Serial.println("  D 0   -> set kd");
  Serial.println("  M 255 -> set max speed command");
  Serial.println("  Z     -> zero encoder");
  Serial.println("  S     -> print status");
  printStatus(readEncoderPositionDeg());
}
 
void loop() {
  processSerial();
 
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlTimeUs) >= CONTROL_PERIOD_US) {
    lastControlTimeUs += CONTROL_PERIOD_US;  // fixed-rate scheduling
    runController();
  }
 
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastDebugTimeMs) >= DEBUG_PERIOD_MS) {
    lastDebugTimeMs += DEBUG_PERIOD_MS;
    printDebug();
  }
}
 
void runController() {
  float deltaT = CONTROL_PERIOD_US / 1.0e6f;
 
  float current_pos = readEncoderPositionDeg();
  float error = target_pos - current_pos;
 
  debug_current_pos = current_pos;
  debug_error = error;
 
  if (!motorEnabled) {
    setMotor(0, 0);
    integral_error = 0.0f;
    prev_error = error;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;
    return;
  }
 
  while (fabsf(error) < MOTOR_MOVEMENT_TOLERANCE_DEG && advanceTargetIfAvailable()) {
    error = target_pos - current_pos;
    integral_error = 0.0f;
    derivative_error = 0.0f;
    prev_error = error;
    debug_error = error;
  }

  if (fabsf(error) < MOTOR_MOVEMENT_TOLERANCE_DEG) {
    setMotor(0, 0);
    integral_error = 0.0f;
    prev_error = error;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;
    return;
  }
 
  integral_error += error * deltaT;
 
  if (integral_error > integral_limit) integral_error = integral_limit;
  if (integral_error < -integral_limit) integral_error = -integral_limit;
 
  derivative_error = (error - prev_error) / deltaT;
  float u = (kp * error) + (ki * integral_error) + (kd * derivative_error);
  prev_error = error;
 
  int dir = 0;
  if (u < 0.0f) {
    dir = 1;
  }
 
  // NEW: better deadband handling
  float abs_u = fabsf(u);
  curVel = 0;

  if (abs_u > DEADBAND_U) {
    curVel = (int)(abs_u - DEADBAND_U);

    if (fabsf(error) > 2.0f && curVel < minSpeedCmd) {
      curVel = minSpeedCmd;
    }
  }

  // curVel = (int)fabsf(u);
 
  // // make sure velocity is less than minSpeedCmd to prevent hitting motor deadband
  // if (curVel > 0 && curVel < minSpeedCmd) {
  // curVel = minSpeedCmd;
  // }
 
 
  // cap velocity at max
  if (curVel > maxSpeedCmd) {
    curVel = maxSpeedCmd;
  } 
 
  // spin the motor
  setMotor(dir, curVel);
 
  debug_u = u;
  debug_vel = curVel;
  debug_dir = dir;
}
 
void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
 
    if (c == '\r') continue;
 
    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';
      if (cmdOverflow) {
        Serial.println("Bad command: line too long.");
      } else if (cmdIndex > 0) {
        handleCommand(cmdBuffer);
      }
      cmdIndex = 0;
      cmdOverflow = false;
    } else {
      if (cmdIndex < (int)sizeof(cmdBuffer) - 1) {
        cmdBuffer[cmdIndex++] = c;
      } else {
        cmdOverflow = true;
      }
    }
  }
}
 
void handleCommand(const char* cmd) {
  while (*cmd == ' ' || *cmd == '\t') {
    cmd++;
  }

  if (*cmd == '\0') {
    return;
  }

  char key = cmd[0];
  const char* args = cmd + 1;
  while (*args == ' ' || *args == '\t') {
    args++;
  }

  // set to zero if z
  if ((key == 'Z' || key == 'z') && *args == '\0') {
    disableMotor();
    setMotor(0, 0);
    clearTargetSequence();
    delay(50);
    setZeroSPI(CS_PIN);
    delay(50);
    target_pos = readEncoderPositionDeg();
    integral_error = 0.0f;
    prev_error = 0.0f;
    Serial.println("Encoder zeroed. Motor disabled. Target set to current position.");
    printStatus(readEncoderPositionDeg());
    return;
  }
 
  // print status
  if ((key == 'S' || key == 's') && *args == '\0') {
    printStatus(readEncoderPositionDeg());
    return;
  }

  if (key == 'T' || key == 't') {
    handleTargetCommand(args);
    return;
  }
 
  float value;
  if (sscanf(cmd, "%c %f", &key, &value) != 2) {
    Serial.println("Bad command.");
    return;
  }
 
  switch (key) {
    case 'P':
    case 'p':
      kp = value;
      Serial.print("kp = ");
      Serial.println(kp, 6);
      break;
 
    case 'I':
    case 'i':
      ki = value;
      integral_error = 0.0f;
      Serial.print("ki = ");
      Serial.println(ki, 6);
      break;
 
    case 'D':
    case 'd':
      kd = value;
      Serial.print("kd = ");
      Serial.println(kd, 6);
      break;
 
    case 'M':
    case 'm':
      maxSpeedCmd = (int)value;
      if (maxSpeedCmd < 0) maxSpeedCmd = 0;
      if (maxSpeedCmd > 255) maxSpeedCmd = 255;
      Serial.print("maxSpeedCmd = ");
      Serial.println(maxSpeedCmd);
      break;
 
    case 'E':
    case 'e':
      if ((int)value == 1) {
        enableMotor();
        clearTargetSequence();
        target_pos = readEncoderPositionDeg();
        integral_error = 0.0f;
        prev_error = 0.0f;
        Serial.println("Motor ENABLED. Target reset to current position.");
      } else {
        disableMotor();
        setMotor(0, 0);
        clearTargetSequence();
        integral_error = 0.0f;
        prev_error = 0.0f;
        Serial.println("Motor DISABLED.");
      }
      break;
 
    default:
      Serial.println("Unknown command.");
      break;
  }
}

void handleTargetCommand(const char* args) {
  float parsedTargets[MAX_TARGET_SEQUENCE];
  int parsedCount = 0;
  int totalCount = 0;
  bool truncated = false;

  const char* p = args;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t') {
      p++;
    }

    if (*p == '\0') {
      break;
    }

    char* endPtr;
    float value = strtof(p, &endPtr);
    if (endPtr == p) {
      Serial.println("Bad target list.");
      return;
    }

    if (parsedCount < MAX_TARGET_SEQUENCE) {
      parsedTargets[parsedCount++] = normalizeTargetAngle(value);
    } else {
      truncated = true;
    }

    totalCount++;
    p = endPtr;
  }

  if (parsedCount == 0) {
    Serial.println("Bad command. Use T angle [angle ...].");
    return;
  }

  for (int i = 0; i < parsedCount; i++) {
    targetSequence[i] = parsedTargets[i];
  }

  targetSequenceCount = parsedCount;
  targetSequenceIndex = 0;
  target_pos = targetSequence[0];

  float current_pos = readEncoderPositionDeg();
  integral_error = 0.0f;
  derivative_error = 0.0f;
  prev_error = target_pos - current_pos;

  Serial.print("target sequence queued: ");
  Serial.print(targetSequenceCount);
  Serial.print(" angle");
  if (targetSequenceCount != 1) Serial.print("s");
  if (truncated) {
    Serial.print(" (first ");
    Serial.print(MAX_TARGET_SEQUENCE);
    Serial.print(" of ");
    Serial.print(totalCount);
    Serial.print(" used)");
  }
  Serial.println();
  Serial.print("target_pos = ");
  Serial.println(target_pos, 3);
}

float normalizeTargetAngle(float angle) {
  if (angle < 0.0f) return 0.0f;
  if (angle >= 360.0f) return fmodf(angle, 360.0f);
  return angle;
}

bool advanceTargetIfAvailable() {
  if (targetSequenceIndex + 1 >= targetSequenceCount) {
    return false;
  }

  targetSequenceIndex++;
  target_pos = targetSequence[targetSequenceIndex];

  Serial.print("target_pos = ");
  Serial.print(target_pos, 3);
  Serial.print(" (");
  Serial.print(targetSequenceIndex + 1);
  Serial.print("/");
  Serial.print(targetSequenceCount);
  Serial.println(")");
  return true;
}

void clearTargetSequence() {
  targetSequenceCount = 0;
  targetSequenceIndex = 0;
}
 
void printDebug() {
  Serial.print("cur=");
  Serial.print(debug_current_pos, 2);
  Serial.print(" tgt=");
  Serial.print(target_pos, 2);
  Serial.print(" err=");
  Serial.print(debug_error, 2);
  Serial.print(" u=");
  Serial.print(debug_u, 2);
  Serial.print(" vel=");
  Serial.print(debug_vel);
  Serial.print(" dir=");
  Serial.print(debug_dir);
  if (targetSequenceCount > 1) {
    Serial.print(" seq=");
    Serial.print(targetSequenceIndex + 1);
    Serial.print("/");
    Serial.print(targetSequenceCount);
  }
  Serial.println();
}
 
void printStatus(float current_pos) {
  Serial.println("----- STATUS -----");
  Serial.print("enabled: ");
  Serial.println(motorEnabled ? "YES" : "NO");
  Serial.print("current_pos: ");
  Serial.println(current_pos, 3);
  Serial.print("target_pos: ");
  Serial.println(target_pos, 3);
  if (targetSequenceCount > 1) {
    Serial.print("target_sequence: ");
    Serial.print(targetSequenceIndex + 1);
    Serial.print("/");
    Serial.println(targetSequenceCount);
  }
  Serial.print("kp: ");
  Serial.println(kp, 6);
  Serial.print("ki: ");
  Serial.println(ki, 6);
  Serial.print("kd: ");
  Serial.println(kd, 6);
  Serial.print("maxSpeedCmd: ");
  Serial.println(maxSpeedCmd);
  Serial.println("------------------");
}
 
void enableMotor() {
  setMotor(0, 0);
  digitalWrite(enable1, HIGH);
  motorEnabled = true;
}
 
void disableMotor() {
  digitalWrite(enable1, LOW);
  motorEnabled = false;
}
 
void setMotor(int dir, int vel) {
  if (vel < 0) vel = 0;
  if (vel > 255) vel = 255;
 
  if (dir == 1) {
    digitalWrite(direction1, LOW);
  } else {
    digitalWrite(direction1, HIGH);
  }
 
  dacWrite(DAC1, vel);
}
 
void setup_encoder() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
}
 
void setup_motor_controller() {
  pinMode(DAC1, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(direction1, OUTPUT);
 
  digitalWrite(direction1, LOW);
  dacWrite(DAC1, 0);
  digitalWrite(enable1, LOW);
}
 
uint16_t readEncoderPosition14Bit(void) {
  uint16_t position = 0;
 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
 
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(3);
 
  position = SPI.transfer(AMT22_NOP);
  position <<= 8;
  delayMicroseconds(3);
 
  position |= SPI.transfer(AMT22_NOP);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  if (verifyChecksumSPI(position)) {
    position &= 0x3FFF;
    return position;
  } 
  else {
    return -1; // sentinel showing failure
  }
 
}
 
float encoderReadingToDeg(uint16_t position) {
  return 360.0f * ((float)position / (NUM_POSITIONS_PER_REV - 1));
}
 
float readEncoderPositionDeg() {
  static float prevAngle = -1;
  uint16_t position = readEncoderPosition14Bit();
  uint16_t negative_one = -1;

  if (position != negative_one) {
    float angle = encoderReadingToDeg(position);
    prevAngle = angle;
    return angle;
  }

  return prevAngle; // just return this for now. TODO: check if we can calculate predictedAngle a different way, if needed
  
  // // this can blow up, curVel is DAC units and micros() is microseconds
  // float predictedAngle = prevAngle + curVel * (micros() - lastControlTimeUs); // TODO: verify this
  // return predictedAngle; 

}
 
void setZeroSPI(uint8_t cs_pin) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
 
  digitalWrite(cs_pin, LOW);
  delayMicroseconds(3);
 
  SPI.transfer(AMT22_NOP);
  delayMicroseconds(3);
 
  SPI.transfer(AMT22_ZERO);
  delayMicroseconds(3);
 
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
 
  delay(250);
}

/*
 * calculate the checksums and then make sure they match what the encoder sent.
 */
bool verifyChecksumSPI(uint16_t message)
{
  //checksum is invert of XOR of bits, so start with 0b11, so things end up inverted
  uint16_t checksum = 0x3;
  for(int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }
  return checksum == (message >> 14);
}
