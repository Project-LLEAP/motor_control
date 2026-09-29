// ESP32/Arduino closed-loop position controller for a motor with an AMT22 encoder.
// The PID loop runs every millisecond and serial commands tune or operate it.

// Hardware SPI support for communicating with the AMT22 encoder.
#include <SPI.h>
// Math functions such as fabs/fabsf used for magnitude and wraparound handling.
#include <math.h>
// C input/output helpers; sscanf parses commands received over Serial.
#include <stdio.h>
// C string helpers; strcmp compares single-letter commands.
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
#define TOLERANCE_STOP 0.5f          // stop when within this
#define TOLERANCE_START 1.0f         // don't restart until outside this
#define CONTROL_PERIOD_US 1000UL     // 1 ms = 1000 Hz
#define DEBUG_PERIOD_MS 100UL        // print at 10 Hz
#define DEADBAND_U 2.0f
#define MAX_QUEUE_SIZE 32

// State: whether the driver is currently allowed to power the motor.
bool motorEnabled = false;

// PID control state: desired angle, previous measured angle, and accumulated terms.
float target_pos = 0.0f;
float prev_pos = 0.0f;
float integral_error = 0.0f;
float derivative_error = 0.0f;

// Tunable PID gains. They can be changed at runtime with P, I, and D commands.
float kp = 5.0f;
float ki = 0.0f;
float kd = 0.0f;

// Bounds for the motor drive command (the ESP32 DAC accepts 0 through 255).
int maxSpeedCmd = 255;
int minSpeedCmd = 30;

// Prevent the integral term from growing without limit while the motor is unable to move.
float integral_limit = 50.0f;

// Derivative low-pass filter coefficient (0.05 to 0.3, lower = more filtering)
float alpha = 0.1f;

// Buffer that collects one line of serial input before it is parsed.
char cmdBuffer[64];
int cmdIndex = 0;

// Timestamps used to run the controller and debug output at fixed intervals.
uint32_t lastControlTimeUs = 0;
uint32_t lastDebugTimeMs = 0;

// A snapshot of the most recent control-cycle values, printed by printDebug().
float debug_current_pos = 0.0f;
float debug_error = 0.0f;
float debug_u = 0.0f;
int debug_vel = 0;
int debug_dir = 0;

// Current output magnitude sent to the motor (named velocity, but it is a DAC command).
float curVel = 0;

// Checksum fault counter
uint32_t checksumFaultCount = 0;

// Optional sequence of target angles. The queue advances after each target is reached.
float posQueue[MAX_QUEUE_SIZE];
int queueLength = 0;
int queueIndex = 0;
bool queueRunning = false;

// Function Defs
void processSerial();
void runController();
void printDebug();
void setup_encoder();
void setup_motor_controller();
void disableMotor();
float readEncoderPositionDeg();
void printStatus();
void disableMotor();
void setMotor();

//================= RTOS Section ====================

// Declare task handles
TaskHandle_t SerialTaskHandle = NULL;
TaskHandle_t ControllerTaskHandle = NULL;
TaskHandle_t DebugTaskHandle = NULL;

//======== RTOS Task Definitions =============

void serialTask(void *parameter){
  for(;;){
    processSerial();
    vTaskDelay(5 / portTICK_PERIOD_MS); //RTOS delay in ticks (5 ms)
  }
}

void controllerTask(void *parameter){
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);  //check every 1 ms

  for (;;) {
    runController();
    vTaskDelayUntil(&lastWake, period);  //delay next task to allow serial & debug to run
  }
}

void debugTask(void *parameter){
  for(;;){
    // Debug output is deliberately much slower, so it does not overwhelm the serial connection.
    printDebug();
    vTaskDelay(50 / portTICK_PERIOD_MS); //RTOS delay in ticks (50 ms)
  }
}


// Arduino startup function: initialize serial, hardware, state, and user instructions.
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

  
  //====== RTOS Setup =======
  if(xTaskCreatePinnedToCore(  //process serial inputs
    serialTask,             // Task function
    "ProcessSerialTask",    // Task name
    10000,                  // Stack size (bytes) - probably much bigger than needed
    NULL,                   // Parameters
    2,                      // Priority
    &SerialTaskHandle,      // Task handle
    1                       // Core 1
  ) != pdPASS){
    Serial.println("Serial Task Creation Failed");
    exit(-1);
  }

  if(xTaskCreatePinnedToCore(  //process motor controller ouput
    controllerTask,         // Task function
    "MotorControllerTask",  // Task name
    10000,                  // Stack size (bytes)
    NULL,                   // Parameters
    3,                      // Priority
    &ControllerTaskHandle,  // Task handle
    1                       // Core 1
  ) != pdPASS){
    Serial.println("Serial Task Creation Failed");
    exit(-1);
  }

  if(xTaskCreatePinnedToCore(  //debugging messages output (for humans)
    debugTask,              // Task function
    "DebugTask",            // Task name
    10000,                  // Stack size (bytes)
    NULL,                   // Parameters
    1,                      // Priority
    &DebugTaskHandle,       // Task handle
    1                       // Core 1
  ) != pdPASS){
    Serial.println("Serial Task Creation Failed");
    exit(-1);
  }
}

//============== Turn this into RTOS :) ==================
//separate process serial task and run controller signal - also separate debug
// in order of importance: controller >> processSerial >> printDebug



// Arduino main loop: always accept commands, then run periodic control and diagnostics.
void loop() {  //Don't need in RTOS framework
  vTaskDelay(1000 / portTICK_PERIOD_MS); //RTOS delay in ticks (1 s)
  // processSerial();

  // // micros() handles short control-loop timing; unsigned subtraction also survives timer wraparound.
  // uint32_t nowUs = micros();
  // if ((uint32_t)(nowUs - lastControlTimeUs) >= CONTROL_PERIOD_US) {
  //   lastControlTimeUs += CONTROL_PERIOD_US;
  //   runController();
  // }

  // // Debug output is deliberately much slower, so it does not overwhelm the serial connection.
  // uint32_t nowMs = millis();
  // if ((uint32_t)(nowMs - lastDebugTimeMs) >= DEBUG_PERIOD_MS) {
  //   lastDebugTimeMs += DEBUG_PERIOD_MS;
  //   printDebug();
  // }
}

// Execute one PID update: read position, calculate an output, and command the motor.
void runController() {
  // Convert the configured control period from microseconds to seconds for PID math.
  float deltaT = CONTROL_PERIOD_US / 1.0e6f;

  // Measure the shaft angle and calculate signed error relative to the desired angle.
  float current_pos = readEncoderPositionDeg();
  float error = current_pos - target_pos;

  // Shortest path around the circle: convert errors beyond half a turn to the equivalent
  // signed error in [-180°, +180°], avoiding nearly full-revolution corrections.
  if (fabs(error) >= 180 && error < 0) {
    error += 360;
  } else if (fabs(error) >= 180 && error > 0) {
    error -= 360;
  }

  debug_current_pos = current_pos;
  debug_error = error;

  // Keep outputs safe and clear accumulated error when the user has disabled the motor.
  if (!motorEnabled) {
    setMotor(0, 0);
    integral_error = 0.0f;
    prev_pos = current_pos;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;
    return;
  }

  // Hysteresis deadband to prevent jitter at target. Entering and leaving use different
  // limits, so minor encoder noise cannot repeatedly restart the motor.
  static bool inDeadband = false;
  if (fabsf(error) < TOLERANCE_STOP) {
    inDeadband = true;
  } else if (fabsf(error) > TOLERANCE_START) {
    inDeadband = false;
  }

  if (inDeadband) {
    setMotor(0, 0);
    // integral_error intentionally NOT reset here to avoid steady state error
    prev_pos = current_pos;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;

    //============ Might run into issues with print timing & serial messages ===============
    // Reaching a queued target immediately selects the next one; otherwise finish the queue.
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

  // Integrate error over time; this is the I component of PID control.
  integral_error += error * deltaT;

  // Clamp the integral term in both directions (anti-windup).
  if (integral_error > integral_limit) integral_error = integral_limit;
  if (integral_error < -integral_limit) integral_error = -integral_limit;

  // Derivative on measurement with low-pass filter to suppress encoder noise
  float raw_derivative = -(current_pos - prev_pos) / deltaT;
  derivative_error = alpha * raw_derivative + (1.0f - alpha) * derivative_error;
  prev_pos = current_pos;

  // Combine proportional, integral, and filtered derivative contributions into one command.
  float u = (kp * error) + (ki * integral_error) + (kd * derivative_error);

  // The sign chooses direction; the absolute value becomes the drive magnitude.
  int dir = 1;
  if (u < 0.0f) {
    dir = 0;
  }

  float abs_u = fabsf(u);
  curVel = 0;

  // Ignore tiny PID outputs that would only cause buzzing or driver noise.
  if (abs_u > DEADBAND_U) {
    curVel = constrain((int)abs_u, 0, maxSpeedCmd);

    if (curVel > 0 && curVel < minSpeedCmd) {
      if (fabsf(error) > 5.0f) {
        curVel = minSpeedCmd;  // enforce min speed only far from target
      } else {
        curVel = 0;            // just stop near target
      }
    }
  }

  setMotor(dir, curVel);

  debug_u = u;
  debug_vel = curVel;
  debug_dir = dir;
}

// Read serial bytes without blocking. A newline marks the end of one command.
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

// Interpret one complete command line received from the Serial Monitor.
void handleCommand(const char* cmd) {
  // Z: write the encoder's current physical position as its new zero reference.
  if (strcmp(cmd, "Z") == 0 || strcmp(cmd, "z") == 0) {
    disableMotor();
    setMotor(0, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS); //RTOS delay in ticks (50 ms)
    setZeroSPI(CS_PIN);
    vTaskDelay(50 / portTICK_PERIOD_MS); //RTOS delay in ticks (50 ms)
    target_pos = readEncoderPositionDeg();
    prev_pos = target_pos;
    integral_error = 0.0f;
    Serial.println("Encoder zeroed. Motor disabled. Target set to current position.");
    printStatus(readEncoderPositionDeg());
    return;
  }

  // S: report the controller's current configuration and position.
  if (strcmp(cmd, "S") == 0 || strcmp(cmd, "s") == 0) {
    printStatus(readEncoderPositionDeg());
    return;
  }

  // X: discard all queued target angles and stop queue execution.
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
    // Skip "T " and walk the line token-by-token to support multiple angle values.
    const char* ptr = cmd + 2;
    while (*ptr != '\0' && queueLength < MAX_QUEUE_SIZE) {
      while (*ptr == ' ') ptr++;
      if (*ptr == '\0') break;
      // Convert the token to a number, then confine targets to one valid revolution.
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

  // All remaining commands have the form: one letter, a space, then a number.
  char key;
  float value;
  if (sscanf(cmd, "%c %f", &key, &value) != 2) {
    Serial.println("Bad command.");
    return;
  }

  // Select the requested setting or enable/disable action.
  switch (key) {
    case 'P':
    case 'p':
      // P adjusts immediate response to position error.
      kp = value;
      Serial.print("kp = ");
      Serial.println(kp, 6);
      break;

    case 'I':
    case 'i':
      // Changing I clears previously accumulated error so old history is not reused.
      ki = value;
      integral_error = 0.0f;
      Serial.print("ki = ");
      Serial.println(ki, 6);
      break;

    case 'D':
    case 'd':
      // D adjusts response to measured motion / rate of change.
      kd = value;
      Serial.print("kd = ");
      Serial.println(kd, 6);
      break;

    case 'M':
    case 'm':
      // Limit maximum motor output to the DAC's supported byte range.
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

// Print a compact periodic telemetry line plus a Serial-Plotter-friendly line.
void printDebug() {
  // Human-readable (Serial Monitor)
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
  Serial.print(" chkfaults=");
  Serial.println(checksumFaultCount);

  // Plotter line — open Serial Plotter to see this
  Serial.print("vel:");
  Serial.print(debug_vel);
  Serial.print(" target:");
  Serial.print(target_pos);
  Serial.print(" pos:");
  Serial.println(debug_current_pos);
}

// Print a fuller on-demand snapshot of the current controller settings and state.
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
  Serial.print("checksumFaults: ");
  Serial.println(checksumFaultCount);
  Serial.println("------------------");
}

// Enable the motor driver after first commanding zero output to avoid a sudden jump.
void enableMotor() {
  setMotor(0, 0);
  digitalWrite(enable1, HIGH);
  motorEnabled = true;
}

// Disable the motor driver's enable pin; setMotor(0, 0) is called separately where needed.
void disableMotor() {
  digitalWrite(enable1, LOW);
  motorEnabled = false;
}

// Apply a direction and 0–255 DAC output value to the motor controller.
void setMotor(int dir, int vel) {
  // Defensively constrain direct callers as well as PID-produced values.
  if (vel < 0) vel = 0;
  if (vel > 255) vel = 255;

  // This wiring maps logical direction 1 to LOW and direction 0 to HIGH.
  if (dir == 1) {
    digitalWrite(direction1, LOW);
  } else {
    digitalWrite(direction1, HIGH);
  }

  // Write the magnitude to the ESP32's DAC output that feeds the motor controller.
  dacWrite(DAC1, vel);
}

// Configure the encoder chip-select line and start the shared SPI peripheral.
void setup_encoder() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
}

// Configure motor-control pins and initialize the driver to a disabled, zero-output state.
void setup_motor_controller() {
  pinMode(DAC1, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(direction1, OUTPUT);

  digitalWrite(direction1, LOW);
  dacWrite(DAC1, 0);
  digitalWrite(enable1, LOW);
}

// Verifies the two checkbits (K1=bit15, K0=bit14) embedded in the raw 16-bit
// encoder response. The AMT22 uses odd parity: K1 covers the odd-indexed
// position bits (1,3,5,7,9,11,13) and K0 covers the even-indexed bits
// (0,2,4,6,8,10,12). Returns true if the position data is valid.
bool verifyChecksumSPI(uint16_t position) {
  bool b[16];
  for (int i = 0; i < 16; i++) {
    b[i] = (position >> i) & 0x01;
  }
  bool k1 = !(b[13] ^ b[11] ^ b[9] ^ b[7] ^ b[5] ^ b[3] ^ b[1]);
  bool k0 = !(b[12] ^ b[10] ^ b[8] ^ b[6] ^ b[4] ^ b[2] ^ b[0]);
  return (b[15] == k1) && (b[14] == k0);
}

// Returns 0xFFFF as a sentinel value on checksum failure.
// Otherwise returns the valid 14-bit position (checkbits already masked off).
// Read one AMT22 SPI frame, validate its parity bits, and return the 14-bit angle count.
uint16_t readEncoderPosition14Bit(void) {
  uint16_t position = 0;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(3);

  // The encoder returns its 16-bit response across two NOP bytes, high byte first.
  position = SPI.transfer(AMT22_NOP);
  position <<= 8;
  delayMicroseconds(3);

  position |= SPI.transfer(AMT22_NOP);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  // Reject a corrupt SPI frame instead of sending a sudden false position to the controller.
  if (!verifyChecksumSPI(position)) {
    checksumFaultCount++;
    return 0xFFFF;  // sentinel: bad read
  }

  // The bottom 14 bits hold the actual position; bits 15 and 14 are checksum bits.
  position &= 0x3FFF;  // discard checkbits, keep 14-bit position
  return position;
}

// Convert an encoder count in the 0–16383 range into an angle in degrees.
float encoderReadingToDeg(uint16_t position) {
  return 360.0f * ((float)position / (NUM_POSITIONS_PER_REV - 1));
}

// On checksum failure, returns the last known good position (debug_current_pos)
// to keep the PID loop stable during a single corrupt sample.
// Get the current angle in degrees. On one invalid sample, reuse the last good reading.
float readEncoderPositionDeg(void) {
  uint16_t raw = readEncoderPosition14Bit();
  if (raw == 0xFFFF) {
    return debug_current_pos;  // hold last good position on bad read
  }
  return encoderReadingToDeg(raw);
}

// Send the AMT22's zero-setting command over SPI, then allow time for it to complete.
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
