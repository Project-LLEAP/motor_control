/**
input as follows: inital postion, inital velocity, final postion, final velocity, duration
*/

#include <SPI.h>

// Motor control defines
#define DAC1 25 //Speed Controller Pin 1
#define DAC2 26 //Speed Controller Pin 2
#define enable1 33 //Motor Enable Pin 1
#define direction1 32 //Motor Direction Pin 1 (HIGH is Positive Direction, LOW is Negative Direction)
#define enable2 23 //Motor Enable Pin 2
#define direction2 19 //Motor Direction Pin 2 (HIGH is Positive Direction, LOW is Negative Direction)

#define gearRatio 25 // Gear Ratio (120.4 w/ Cycloidal, 28, + Planetary, 4.3)
#define maxVel 1000.0 // Maximum Velocity of Motor (RPM)
#define minVel 0.0 // Minimum Velocity of Motor (RPM)
#define maxVolt 3.3 // Maximum Voltage from ESP32 
#define minVolt 0.1 // Minimum Voltage accepted by Motor Driver

// encoder defines
#define CS_PIN 5           // Chip select connected to digital pin 2
#define AMT22_NOP 0x00     // No-operation byte per datasheet
#define NUM_POSITIONS_PER_REV 16384  // 2^14 bit encoder
#define AMT22_ZERO 0x70

int sign = 1; // Create Global sign variable and initialize to 1

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
  printEncoderPosition(position_14bit, position_float);

  // main code to run repeatedly:
  if (Serial.available() > 0) { //Check if Serial Messages Were Recieved
    String input = Serial.readString();
    int index = input.indexOf(' ');
    //Create Strings to Later Print to Serial
    String del_x_s = "";
    String dir_s = "";
    String reset_s = "";

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
        }
     }
    }
    //Convert All Strings to Floats
    float del_x = del_x_s.toFloat();
    int dir = dir_s.toInt();
    int reset = reset_s.toInt();

    // confirm inputs read properly
    Serial.print("del_x: ");
    Serial.println(del_x, DEC);            // Print absolute position value
    Serial.print("dir: ");
    Serial.println(dir, DEC);
    Serial.print("reset: ");
    Serial.println(reset, DEC);
    delay(1000);

    moveToAngle(del_x, dir);

    // if "reset" is passed in as true (1), just go back to initial position (roughly)
    if (reset == 1) {
      delay(5000);
      dir ^= 1;
      moveToAngle(del_x, dir);
    }
    delay(1000);
  }
}

void moveToAngle(float targetAngle, bool dir) {
  digitalWrite(enable1, HIGH);

  // Change Direction based on Velocity Sign
  if (dir == 0){ // Switch to negative direction
    digitalWrite(direction1, LOW);
    sign = -1;
  } else { // Switch to positive direction1 when x2 is greater than x1
    digitalWrite(direction1, HIGH);
    sign = 1;
  }

  dacWrite(DAC1, 127);  // arbitrary constant speed for testing

  uint16_t position_14bit = readEncoderPosition14Bit();
  float position_float = encoderReadingToDeg(position_14bit);
  // run at constant vel until encoder reads targetAngle, then stop motor
  while (position_float < targetAngle) {
    printEncoderPosition(position_14bit, position_float);
    position_14bit = readEncoderPosition14Bit();
    position_float = encoderReadingToDeg(position_14bit);
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

