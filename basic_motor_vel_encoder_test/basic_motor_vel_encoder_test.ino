/**
input as follows: target angle [0, 360)(deg), direction (0 or 1), reset (0 or 1), vel [0-255]
*/

#include <SPI.h>

// Motor control defines
#define DAC1 25 //Speed Controller Pin 1
#define enable1 33 //Motor Enable Pin 1
#define direction1 32 //Motor Direction Pin 1 (HIGH is Positive Direction, LOW is Negative Direction)

// encoder defines
#define CS_PIN 5           // Chip select connected to digital pin 2
#define AMT22_NOP 0x00     // No-operation byte per datasheet
#define NUM_POSITIONS_PER_REV 16384  // 2^14 bit encoder
#define AMT22_ZERO 0x70
#define MOTOR_MOVEMENT_TOLERANGE_DEG 10 

void setup() {
  Serial.begin(115200); //Start Serial Communication Rate at This Value
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

    if (index != -1) { //Check if There is a Space Character
      del_x_s = input.substring(0, index); //Create a String by Copying From 0 to Position of First Space
     
      if (index != -1) {
        input = input.substring(index+1); //Redefine Input Beginning After Indexed Space to End of Previous Input
        index = input.indexOf(' ');
        dir_s = input.substring(0, index); //Repeat for All Variables
        
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
    int vel_8bit = vel_8bit_s.toInt();

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

/*

- targetAngle is the absolute angle we want it to move to
- we knew our current position with position_float, which is updated every while loop
- given a current position and a targetAngle, stop when the current position reaches targetAngle

we have three versions. 
1. keep running until the current read angle reaches the target angle (with tolerance)

      the most straightforward logic, which we tried first.
      this didn't work previously, but we didn't try it with tolerance
      if it stops with tolerance value, then the problem is probably the sampling rate not the code

2. cumulative angle is a calculated relative angle moved, found by adding small deltas together

      the point is to check delta for whether the delta is huge.
      the error grows infinitely large for the negative direction

      now that i'm looking at this again...
      what's the difference between cumulative angle and the current read position? aren't they the same, 
        except cumulative angle is less accurate due to sampling rate?
      i think the goal was to make logic that would work for both directions, and then we added a tolerance value

      i dont have the updated code, so ill put this on hold for now
            
3. create a break point (this version)

      four cases:
        CLOCKWISE
        1. target angle > start angle
        2. target angle < start angle
        COUNTER-CLOCKWISE
        3. target angle < start angle
        4. target angle > start angle

      my only problem is that creating a break point doubles the margin of error. what if the encoder never 
        reads the breakpoint either? mega cooked
      this will NOT solve the problem if the sampling rate is the problem
      test the first version of this code first WITH A TOLERANCE to check whether its the sampling rate only

*/



void moveToAngle(float targetAngle, int dir, int vel_8bit) {

  // turn the motor on
  digitalWrite(enable1, HIGH);

  // Change Direction based on Velocity Sign
  if (dir == 0) { digitalWrite(direction1, LOW); } // Switch to negative direction 
  else { digitalWrite(direction1, HIGH); } // Switch to positive direction1 when x2 is greater than x1

  dacWrite(DAC1, vel_8bit);  // arbitrary constant speed for testing

  uint16_t position_14bit = readEncoderPosition14Bit();
  float position_float = encoderReadingToDeg(position_14bit); // our current position

  // run at constant vel until encoder reads targetAngle, then stop motor

  while (true) {
    printEncoderPosition(position_14bit, position_float);
    position_14bit = readEncoderPosition14Bit();
    position_float = encoderReadingToDeg(position_14bit);

    float error = targetAngle - position_float;
    if (abs(error) < MOTOR_MOVEMENT_TOLERANCE_DEG) {
      break;
    }
  }
  dacWrite(DAC1, 0);
  
  // shutoff the motor
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

