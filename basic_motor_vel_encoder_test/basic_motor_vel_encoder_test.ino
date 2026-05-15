#include <SPI.h>
#include <math.h>
#include <stdio.h>
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
#define TOLERANCE_STOP 1.5f          // stop when within this
#define TOLERANCE_START 3.0f         // don't restart until outside this
#define CONTROL_PERIOD_US 1000UL     // 1 ms = 1000 Hz
#define DEBUG_PERIOD_MS 100UL        // print at 10 Hz
#define DEADBAND_U 2.0f
#define MAX_QUEUE_SIZE 32
 
// State
bool motorEnabled = false;
 
// Control state
float target_pos = 0.0f;
float prev_pos = 0.0f;
float integral_error = 0.0f;
float derivative_error = 0.0f;
 
// Tunable gains
float kp = 5.0f;
float ki = 0.0f;
float kd = 0.0f;
 
// Tunable max command
int maxSpeedCmd = 255;
int minSpeedCmd = 25;
 
// Anti-windup clamp
float integral_limit = 100.0f;
 
// Serial command buffer
char cmdBuffer[64];
int cmdIndex = 0;
 
// Timing
uint32_t lastControlTimeUs = 0;
uint32_t lastDebugTimeMs = 0;
 
// Debug values
float debug_current_pos = 0.0f;
float debug_error = 0.0f;
float debug_u = 0.0f;
int debug_vel = 0;
int debug_dir = 0;
 
float curVel = 0;
 
float posQueue[MAX_QUEUE_SIZE];
int queueLength = 0;
int queueIndex = 0;
bool queueRunning = false;
 
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
  prev_pos = target_pos;
 
  lastControlTimeUs = micros();
  lastDebugTimeMs = millis();
 
  Serial.println("Controller ready.");
  Serial.println("Motor is DISABLED.");
  Serial.println("Commands:");
  Serial.println("  E 1       -> enable motor");
  Serial.println("  E 0       -> disable motor");
  Serial.println("  T 45      -> set target angle");
  Serial.println("  T 0 45 90 -> queue multiple angles");
  Serial.println("  X         -> clear queue");
  Serial.println("  P 5       -> set kp");
  Serial.println("  I 0       -> set ki");
  Serial.println("  D 0       -> set kd");
  Serial.println("  M 255     -> set max speed command");
  Serial.println("  Z         -> zero encoder");
  Serial.println("  S         -> print status");
  printStatus(readEncoderPositionDeg());
}
 
void loop() {
  processSerial();
 
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlTimeUs) >= CONTROL_PERIOD_US) {
    lastControlTimeUs += CONTROL_PERIOD_US;
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
 
  if (fabs(error) >= 180 && error < 0) {
    error += 360;
  } else if (fabs(error) >= 180 && error > 0) {
    error -= 360;
  }
 
  debug_current_pos = current_pos;
  debug_error = error;
 
  if (!motorEnabled) {
    setMotor(0, 0);
    integral_error = 0.0f;
    prev_pos = current_pos;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;
    return;
  }
 
  // Hysteresis deadband to prevent jitter at target
  static bool inDeadband = false;
  if (fabsf(error) < TOLERANCE_STOP) {
    inDeadband = true;
  } else if (fabsf(error) > TOLERANCE_START) {
    inDeadband = false;
  }
 
  if (inDeadband) {
    setMotor(0, 0);
    integral_error = 0.0f;
    prev_pos = current_pos;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;
 
    if (queueRunning && queueIndex < queueLength) {
      target_pos = posQueue[queueIndex++];
      inDeadband = false;
      integral_error = 0.0f;
      Serial.print("Queue -> ");
      Serial.println(target_pos, 2);
    } else if (queueRunning && queueIndex >= queueLength) {
      queueRunning = false;
      Serial.println("Queue complete.");
    }
    return;
  }
 
  integral_error += error * deltaT;
 
  if (integral_error > integral_limit) integral_error = integral_limit;
  if (integral_error < -integral_limit) integral_error = -integral_limit;
 
  // Derivative on measurement to avoid derivative kick on setpoint change
  derivative_error = -(current_pos - prev_pos) / deltaT;
  prev_pos = current_pos;
 
  float u = (kp * error) + (ki * integral_error) + (kd * derivative_error);
 
  int dir = 0;
  if (u < 0.0f) {
    dir = 1;
  }
 
  float abs_u = fabsf(u);
  curVel = 0;
 
  if (abs_u > DEADBAND_U) {
    curVel = constrain((int)abs_u, 0, maxSpeedCmd);
 
    if (curVel > 0 && curVel < minSpeedCmd) {
      if (fabsf(error) > 5.0f) {
        curVel = minSpeedCmd;
      } else {
        curVel = 0;
      }
    }
  }
 
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
      if (cmdIndex > 0) {
        handleCommand(cmdBuffer);
      }
      cmdIndex = 0;
    } else {
      if (cmdIndex < (int)sizeof(cmdBuffer) - 1) {
        cmdBuffer[cmdIndex++] = c;
      }
    }
  }
}
 
void handleCommand(const char* cmd) {
  if (strcmp(cmd, "Z") == 0 || strcmp(cmd, "z") == 0) {
    disableMotor();
    setMotor(0, 0);
    delay(50);
    setZeroSPI(CS_PIN);
    delay(50);
    target_pos = readEncoderPositionDeg();
    prev_pos = target_pos;
    integral_error = 0.0f;
    Serial.println("Encoder zeroed. Motor disabled. Target set to current position.");
    printStatus(readEncoderPositionDeg());
    return;
  }
 
  if (strcmp(cmd, "S") == 0 || strcmp(cmd, "s") == 0) {
    printStatus(readEncoderPositionDeg());
    return;
  }
 
  if (strcmp(cmd, "X") == 0 || strcmp(cmd, "x") == 0) {
    queueRunning = false;
    queueLength = 0;
    queueIndex = 0;
    Serial.println("Queue cleared.");
    return;
  }
 
  // T command handled before sscanf so multiple angles are parsed correctly
  if (cmd[0] == 'T' || cmd[0] == 't') {
    queueLength = 0;
    queueIndex = 0;
    queueRunning = false;
    const char* ptr = cmd + 2;
    while (*ptr != '\0' && queueLength < MAX_QUEUE_SIZE) {
      while (*ptr == ' ') ptr++;
      if (*ptr == '\0') break;
      float val = atof(ptr);
      posQueue[queueLength++] = constrain(val, 0.0f, 359.9f);
      while (*ptr != ' ' && *ptr != '\0') ptr++;
    }
    if (queueLength == 1) {
      target_pos = posQueue[0];
      queueLength = 0;
      integral_error = 0.0f;
      Serial.print("target_pos = ");
      Serial.println(target_pos, 3);
    } else {
      queueIndex = 0;
      queueRunning = true;
      target_pos = posQueue[queueIndex++];
      integral_error = 0.0f;
      prev_pos = readEncoderPositionDeg();
      Serial.print("Queue loaded: ");
      Serial.print(queueLength);
      Serial.println(" points.");
    }
    return;
  }
 
  char key;
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
        target_pos = readEncoderPositionDeg();
        prev_pos = target_pos;
        integral_error = 0.0f;
        Serial.println("Motor ENABLED. Target reset to current position.");
      } else {
        disableMotor();
        setMotor(0, 0);
        integral_error = 0.0f;
        Serial.println("Motor DISABLED.");
      }
      break;
 
    default:
      Serial.println("Unknown command.");
      break;
  }
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
  Serial.println(debug_dir);
 
  Serial.print("vel:");
  Serial.print(debug_vel);
  Serial.print(" err:");
  Serial.println(debug_error);
}
 
void printStatus(float current_pos) {
  Serial.println("----- STATUS -----");
  Serial.print("enabled: ");
  Serial.println(motorEnabled ? "YES" : "NO");
  Serial.print("current_pos: ");
  Serial.println(current_pos, 3);
  Serial.print("target_pos: ");
  Serial.println(target_pos, 3);
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
    digitalWrite(direction1, HIGH);
  } else {
    digitalWrite(direction1, LOW);
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
 
  position &= 0x3FFF;
  return position;
}
 
float encoderReadingToDeg(uint16_t position) {
  return 360.0f * ((float)position / (NUM_POSITIONS_PER_REV - 1));
}
 
float readEncoderPositionDeg(void) {
  return encoderReadingToDeg(readEncoderPosition14Bit());
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