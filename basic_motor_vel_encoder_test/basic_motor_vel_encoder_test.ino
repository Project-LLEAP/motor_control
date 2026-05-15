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
#define MOTOR_MOVEMENT_TOLERANCE_DEG 0.5f
#define DEADBAND_U 1.0f
#define CONTROL_PERIOD_US 2000UL     // 2 ms = 500 Hz
#define DEBUG_PERIOD_MS 100UL        // print at 10 Hz
 
// State
bool motorEnabled = false;
 
// Control state
float target_pos = 0.0f;
float prev_error = 0.0f;
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
float integral_limit = 50.0f;
 
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
 
/*
want the controller to advance to the next waypoint once it is close enough, while keeping the motor enabled and the PID loop running continuously
waypoint array: array of points to move to
*/

#define MAX_WAYPOINTS 20

struct Waypoint {
  float targetDeg;
  float kp;
  float ki;
  float kd;
};

Waypoint waypoints[MAX_WAYPOINTS];
int numWaypoints = 0;
int currentWaypoint = 0;
bool sequenceActive = false;

void loadCurrentWaypoint() {
  if (currentWaypoint < 0 || currentWaypoint >= numWaypoints) {
    sequenceActive = false;
    return;
  }

  target_pos = waypoints[currentWaypoint].targetDeg;
  kp = waypoints[currentWaypoint].kp;
  ki = waypoints[currentWaypoint].ki;
  kd = waypoints[currentWaypoint].kd;

  integral_error = 0.0f;
  prev_error = 0.0f;

  Serial.print("Moving to waypoint ");
  Serial.print(currentWaypoint);
  Serial.print(": target=");
  Serial.print(target_pos, 3);
  Serial.print(" kp=");
  Serial.print(kp, 6);
  Serial.print(" ki=");
  Serial.print(ki, 6);
  Serial.print(" kd=");
  Serial.println(kd, 6);
}

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
  Serial.println("  T 45  -> set target angle");
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

float angleErrorDeg(float target, float current){
  float error = target-current;

  while(error >180.0f) error -= 360.0f;
  while(error <-180.0f) error += 360.0f;
  return error;

}


void runController() {
  float deltaT = CONTROL_PERIOD_US / 1.0e6f;
 
  float current_pos = readEncoderPositionDeg();
  //TODO--> by jason :)
  //float error = target_pos - current_pos;
  float error= angleErrorDeg(target_pos, current_pos);
 
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
 
  if (fabsf(error) < MOTOR_MOVEMENT_TOLERANCE_DEG) {
  integral_error = 0.0f;
  prev_error = error;

  if (sequenceActive) {
    currentWaypoint++;

    if (currentWaypoint < numWaypoints) {
      loadCurrentWaypoint();
      return;
    } else {
      sequenceActive = false;
      setMotor(0, 0);
      debug_u = 0.0f;
      debug_vel = 0;
      debug_dir = 0;
      Serial.println("Waypoint sequence complete.");
      return;
    }
  }

  setMotor(0, 0);
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
 
  float abs_u = fabsf(u);
  int vel = 0;

  if (abs_u > DEADBAND_U) {
    vel = (int)abs_u;

    if (vel > 0 && vel < minSpeedCmd) {
      vel = minSpeedCmd;
    }
  }
 
  if (vel > maxSpeedCmd) {
    vel = maxSpeedCmd;
  }
 
  setMotor(dir, vel);
 
  debug_u = u;
  debug_vel = vel;
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
    integral_error = 0.0f;
    prev_error = 0.0f;
    Serial.println("Encoder zeroed. Motor disabled. Target set to current position.");
    printStatus(readEncoderPositionDeg());
    return;
  }
 
  if (strcmp(cmd, "S") == 0 || strcmp(cmd, "s") == 0) {
    printStatus(readEncoderPositionDeg());
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
 
    case 'T':
    case 't':
      target_pos = value;
      if (target_pos < 0.0f) target_pos = 0.0f;
      if (target_pos >= 360.0f) target_pos = fmodf(target_pos, 360.0f);
      Serial.print("target_pos = ");
      Serial.println(target_pos, 3);
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
        integral_error = 0.0f;
        prev_error = 0.0f;
        Serial.println("Motor ENABLED. Target reset to current position.");
      } else {
        disableMotor();
        setMotor(0, 0);
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

// checksum code did not work. above is the working code with deadband, and below is the checksum implementation commented out

// uint16_t readEncoderPosition14Bit(void) {
//   uint16_t position = 0;
 
//   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
 
//   digitalWrite(CS_PIN, LOW);
//   delayMicroseconds(3);
 
//   position = SPI.transfer(AMT22_NOP);
//   position <<= 8;
//   delayMicroseconds(3);
 
//   position |= SPI.transfer(AMT22_NOP);
//   digitalWrite(CS_PIN, HIGH);
//   SPI.endTransaction();

//   if (verifyChecksumSPI(position)) {
//     position &= 0x3FFF;
//     return position;
//   } 
//   else {
//     return -1; // sentinel showing failure
//   }
 
// }

// /*
//  * calculate the checksums and then make sure they match what the encoder sent.
//  */
// bool verifyChecksumSPI(uint16_t message)
// {
//   //checksum is invert of XOR of bits, so start with 0b11, so things end up inverted
//   uint16_t checksum = 0x3;
//   for(int i = 0; i < 14; i += 2)
//   {
//     checksum ^= (message >> i) & 0x3;
//   }
//   return checksum == (message >> 14);
// }

