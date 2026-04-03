/**
input as follows: target angle [0, 360)(deg), direction (0 or 1), reset (0 or 1), vel [0-255]
*/

#include <SPI.h>

// Motor control defines
#define DAC1 25 // Speed Controller Pin 1
#define enable1 33 // Motor Enable Pin 1
#define direction1 32 // Motor Direction Pin 1 (HIGH is Positive Direction, LOW is Negative Direction)

// encoder defines
#define CS_PIN 5           // Chip select connected to digital pin 2
#define AMT22_NOP 0x00     // No-operation byte per datasheet
#define NUM_POSITIONS_PER_REV 16384  // 2^14 bit encoder
#define AMT22_ZERO 0x70
#define MOTOR_MOVEMENT_TOLERANCE_DEG 2.0f    // Error margine to check the angle
#define DECEL_ZONE_DEG               15.0f   // start slowing down 15° before target
#define MIN_VEL_8BIT                 30      // minimum voltage to keep motor moving

// PID variables
#define KP              2.0f    // k proportional 
#define KI              0.0f    // start at zero -- accumulated error
#define KD              0.0f    // start at zero -- how fast error is changing right now
#define MAX_DELTA_DEG  5.0f  // tune this — start conservative

#define INTEGRAL_LIMIT  50.0f

#define LOOP_DELAY_MS                5       // yield to ESP32 watchdog

void setup() {
  Serial.begin(115200); // Start Serial Communication Rate at This Value
  delay(1000);

  // setup pins for encoder and motor controller
  setup_encoder();
  setup_motor_controller();

  setZeroSPI(CS_PIN);
}

void printEncoderPosition(uint16_t position_14bit, float position_float) {
  Serial.print("Position (14 bit): ");
  Serial.print(position_14bit, DEC);            // Print absolute position value
  Serial.print(" = ");
  Serial.print(position_float, DEC);
  Serial.println(" deg");
}

void loop() {
  delay(500);
  uint16_t position_14bit = readEncoderPosition14Bit();
  float position_float = encoderReadingToDeg(position_14bit);
  Serial.print("In main loop (not in moveToAngle)");
  printEncoderPosition(position_14bit, position_float);

  // main code to run repeatedly:
  if (Serial.available() > 0) { //Check if Serial Messages Were Recieved
    String input = Serial.readString();
    int index = input.indexOf(' ');
    //Create Strings to Later Print to Serial
    String del_x_s = "";
    String dir_s = "";
    String reset_s = "";
    String vel_8bit_s = "";

    if (index != -1) { // Check if There is a Space Character
      del_x_s = input.substring(0, index); // Create a String by Copying From 0 to Position of First Space
     
      if (index != -1) {
        input = input.substring(index+1); // Redefine Input Beginning After Indexed Space to End of Previous Input
        index = input.indexOf(' ');
        dir_s = input.substring(0, index); // Repeat for All Variables
        
        if (index != -1) {    
          input = input.substring(index+1);
          index = input.indexOf(' ');
          reset_s = input.substring(0, index);
          if (index != -1) {    
            input = input.substring(index+1);
            index = input.indexOf(' ');
            vel_8bit_s = input.substring(0, index);
        }
        }
     }
    }
    //Convert All Strings to Floats
    float del_x = del_x_s.toFloat();
    int dir = dir_s.toInt();
    int reset = reset_s.toInt();
    //int vel_8bit = vel_8bit_s.toInt();
    int vel_8bit = 175;

    // confirm inputs read properly
    Serial.print("del_x: ");
    Serial.println(del_x, DEC);            // Print absolute position value
    Serial.print("dir: ");
    Serial.println(dir, DEC);
    Serial.print("reset: ");
    Serial.println(reset, DEC);
    Serial.print("vel_8bit: ");
    Serial.println(vel_8bit, DEC);
    delay(1000);

    moveToAngle(del_x, dir, vel_8bit);

    // if "reset" is passed in as true (1), just go back to initial position (roughly)
    if (reset == 1) {
      delay(5000);
      dir ^= 1;
      moveToAngle(del_x, dir, vel_8bit);
    }
    delay(1000);
  }
}

int computePID(float error, float deltaTime, float &integral, float &lastError) {
  float P          = KP * error;

  integral        += error * deltaTime;
  if (integral >  INTEGRAL_LIMIT) integral =  INTEGRAL_LIMIT;
  if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
  float I          = KI * integral;

  float derivative = (error - lastError) / deltaTime;
  float D          = KD * derivative;
  lastError        = error;

  float output     = P + I + D;
  int command_vel  = (int)abs(output);
  if (command_vel < MIN_VEL_8BIT) command_vel = MIN_VEL_8BIT;
  if (command_vel > 255)          command_vel = 255;

  return command_vel;
}

void moveToAngle(float targetAngle, int dir, int vel_8bit) {

  digitalWrite(enable1, HIGH);
  if (dir == 0) { digitalWrite(direction1, LOW);  }
  else          { digitalWrite(direction1, HIGH); }

  uint16_t position_14bit = readEncoderPosition14Bit();
  if (position_14bit == 0xFFFF) {
    Serial.println("ERROR: Bad initial encoder read. Aborting.");
    digitalWrite(enable1, LOW);
    return;
  }

  float lastRawAngle    = encoderReadingToDeg(position_14bit);
  float cumulativeAngle = 0;

  // PID state
  float integral         = 0.0f;
  float lastError        = targetAngle;
  unsigned long lastTime = millis();

  dacWrite(DAC1, vel_8bit);  // initial kick

  while (true) {

    position_14bit = readEncoderPosition14Bit();
    if (position_14bit == 0xFFFF) { delay(LOOP_DELAY_MS); continue; }

    float position_float = encoderReadingToDeg(position_14bit);

    float delta = position_float - lastRawAngle;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    // ✅ Spike filter — reject physically impossible deltas
    if (abs(delta) > MAX_DELTA_DEG) {
      Serial.print("WARNING: Spike rejected, delta=");
      Serial.println(delta);
      lastRawAngle = position_float;  // update reference but don't accumulate
      delay(LOOP_DELAY_MS);
      continue;  // skip this iteration entirely
    }

    cumulativeAngle += delta;
    lastRawAngle     = position_float;

    float error = targetAngle - cumulativeAngle;

    // Delta time
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = LOOP_DELAY_MS / 1000.0f;
    lastTime = currentTime;

   
    // Stop condition
    if (abs(error) < MOTOR_MOVEMENT_TOLERANCE_DEG) {
      Serial.println("Target reached!");
      break;
    }

    // PID controls speed
    dacWrite(DAC1, computePID(error, dt, integral, lastError));

    Serial.print("Cumulative: "); Serial.print(cumulativeAngle);
    Serial.print(" | Error: ");   Serial.print(error);
    delay(LOOP_DELAY_MS);
  }

  dacWrite(DAC1, 0);
  digitalWrite(enable1, LOW);
}

void setup_encoder() {
  pinMode(CS_PIN, OUTPUT);     // Set chip select pin
  SPI.begin();                 // Initialize SPI bus
  digitalWrite(CS_PIN, HIGH);  // Default CS high (inactive)
}

void setup_motor_controller() {
  pinMode(DAC1, OUTPUT); //Push a DAC Output to Motor Speed Controller
  pinMode(enable1, OUTPUT); //Push an Enable Signal Output to Motor Controller
  pinMode(direction1, OUTPUT); //Push a Direction Signal Output to Motor Controller
  digitalWrite(enable1, LOW);
}

uint16_t readEncoderPosition14Bit(void) {
  uint16_t position = 0;

  digitalWrite(CS_PIN, LOW);           // Begin SPI: CS Low
  delayMicroseconds(3);                // Wait >= 3us per protocol

  position = SPI.transfer(AMT22_NOP);  // First byte (High byte)
  position = position << 8;
  delayMicroseconds(3);

  position |= SPI.transfer(AMT22_NOP); // Second byte (Low byte)
  digitalWrite(CS_PIN, HIGH);          // End SPI: CS High

  // Mask for upper two checksum bits (position valid bits are 0-13)
  position &= 0x3FFF;
  return position;
}

float encoderReadingToDeg(uint16_t position) {
  return 360 * ((float)position / (NUM_POSITIONS_PER_REV-1));
}

float readEncoderPositionDeg(void) {
  return encoderReadingToDeg(readEncoderPosition14Bit());
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t cs_pin)
{
  //set CS to low
  digitalWrite(cs_pin, LOW);
  delayMicroseconds(3);

  //send the first byte of the command
  SPI.transfer(AMT22_NOP);
  delayMicroseconds(3);

  //send the second byte of the command
  SPI.transfer(AMT22_ZERO);
  delayMicroseconds(3);
  
  //set CS to high
  digitalWrite(cs_pin, HIGH);

  delay(250); //250 millisecond delay to allow the encoder to reset
}