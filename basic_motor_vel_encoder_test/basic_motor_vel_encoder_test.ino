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

float coeff[4]; //Create Global Coefficienct Matrix
int sign = 1; // Create Global sign variable and initialize to 1

void setup() {
  Serial.begin(115200); //Start Serial Communication Rate at This Value
  delay(1000);

  // setup pins for encoder and motor controller
  setup_encoder();
  setup_motor_controller();

  setZeroSPI(CS_PIN);

  /**
  input as follows: inital postion, inital velocity, final postion, final velocity, duration
  */
  // float points[][5] = {{0,0,45,0,5}}; //, // gives -0.72x^{3}+5.4x^{2}
  //                      // {45,0,0,0,5}};
  // int pointNumber = sizeof(points)/sizeof(points[0]);

  // motionProfile(points, pointNumber);
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
    String x1_s = "";
    String v1_s = "";
    String x2_s = "";
    String v2_s = "";
    String duration_s = "";

    if (index != -1){ //Check if There is a Space Character
     x1_s = input.substring(0, index); //Create a String by Copying From 0 to Position of First Space
     
     if (index != -1){
       input = input.substring(index+1); //Redefine Input Beginning After Indexed Space to End of Previous Input
       index = input.indexOf(' ');
       v1_s = input.substring(0, index); //Repeat for All Variables
       
       if (index != -1){    
         input = input.substring(index+1);
         index = input.indexOf(' ');
         x2_s = input.substring(0, index);
         
         if (index != -1){    
           input = input.substring(index+1);
           index = input.indexOf(' ');
           v2_s = input.substring(0, index);
           
           if (index != -1){    
             input = input.substring(index+1);
             index = input.indexOf(' ');
             duration_s = input.substring(0, index);
           }
         }
       }
     }
    }
    //Convert All Strings to Floats
    float x1 = x1_s.toFloat();
    float v1 = v1_s.toFloat();
    float x2 = x2_s.toFloat();
    float v2 = v2_s.toFloat();
    float duration = duration_s.toFloat();
    float points[][5] = {x1,v1,x2,v2,duration};

    motionProfile(points, 1);
    delay(1000);
  }
}

// Input an array of points to hit and the number of points in the array
void motionProfile(float points[][5], int pointNumber) {
  digitalWrite(enable1, HIGH);
  float c[pointNumber][4]; // Initialize the c 2D-array
  for (int i = 0; i < pointNumber; i++) { // Loop through all the points
    // Solve the 4x4 Matrix to get polynomial coefficients
    matrixSolver(gearRatio*points[i][0],gearRatio*points[i][1],0,\
    gearRatio*points[i][2],gearRatio*points[i][3],points[i][4]);

    // Set current c array row to current coefficients
    c[i][0] = coeff[0];
    c[i][1] = coeff[1];
    c[i][2] = coeff[2];
    c[i][3] = coeff[3];
  }

  // Loop through all points again
  for (int i = 0; i < pointNumber; i++) {
    float t = 0; // Inititalize t
    float positionOut = points[i][0]; // Initialize positionOut
    float tOld = 0; // Initialize tOld
    
    float c1 = c[i][0];
    float c2 = c[i][1];
    float c3 = c[i][2];
    float c4 = c[i][3];

    float tStart = micros()*pow(10,-6); // Note the start time for relative time calculations
    while (t < points[i][4]) { // While the current time is less than the end time
      t = (micros()*pow(10,-6))-tStart; // Set new current relative time
      // Calculate what the current velocity should be
      float vel = 3*c1*pow(t,2) + 2*c2*t + c3; // Current Velocity in degs/s
      float velocity = vel/6; // Convert from degs/s to RPM

      // Change Direction based on Velocity Sign
      if (velocity < 0){ // Switch to negative direction1 when x2 is less than x1
      digitalWrite(direction1, LOW);
      sign = -1;
      }
      if (velocity > 0){ // Switch to positive direction1 when x2 is greater than x1
      digitalWrite(direction1, HIGH);
      sign = 1;
      }
      
      // Convert RPM to a Voltage
      float volts = minVolt + (abs(velocity) - 0.0)*((maxVolt - minVolt)/(maxVel-minVel));
      // Covert Voltage to DAC1 Output
      int dacOut = round(0.0 + (volts - 0.0)*((255 - 0)/(maxVolt-0.0)));
      
      float voltsOut = 0.0 + (dacOut - 0.0)*((maxVolt-0.0)/(255-0)); // Map dacOut to "actual" Voltage
      float velocityOut = 0.0 + (voltsOut - minVolt)*((maxVel-minVel)/(maxVolt-minVolt)); // Map "actual" Voltage to "actual" Velocity

      positionOut = positionOut + sign*(velocityOut*6)*(t-tOld)/gearRatio; // Integrate "actual" velocity over time to get "actual" position
      float position = (c1*pow(t,3) + c2*pow(t,2) + c3*t + c4)/gearRatio; //  Caluclated Joint Position

      // Check if the DAC1 Output is Outside of its Acceptable Range
      if (dacOut < 0 || dacOut > 255) {
        dacOut = max(min(255, dacOut), 0); // If out of the Acceptable Range
      }

      dacWrite(DAC1, dacOut);

      Serial.print(t, 5);
      Serial.print(",");
      Serial.print(velocity);
      Serial.print(",");
      Serial.print(dacOut);
      Serial.print(",");
      Serial.print(position);
      Serial.print(",");
      Serial.print(positionOut);
      Serial.print(",");
      Serial.print(voltsOut);
      Serial.println();
      tOld = t;
      
      // encoder reading during trajectory
      uint16_t position_14bit = readEncoderPosition14Bit();
      float position_float = encoderReadingToDeg(position_14bit);
      printEncoderPosition(position_14bit, position_float);
      float pos_error = position_float - positionOut;
      Serial.print("Position error (encoder reading - positionOut) = ");
      Serial.println(pos_error);
    }
    dacWrite(DAC1, 0);
  }
  digitalWrite(enable1, LOW);
}

/* Solve 4x4 Matrix with Cramer's Rule to 
  Determine the Polynomial Constants Required to Generate a Cubic Position Function*/
void matrixSolver(float x1, float v1, float t1, float x2, float v2, float t2) {
  // 1st Equation/Row
  float a = pow(t1,3);
  float b = pow(t1,2);
  float c = t1;
  float d = 1;
  float q = x1;

  // 2nd Equation/Row
  float e = pow(t2,3);
  float f = pow(t2,2);
  float g = t2;
  float h = 1;
  float r = x2;

  // 3rd Equation/Row
  float i = 3*pow(t1,2);
  float j = 2*t1;
  float k = 1;
  float l = 0;
  float s = v1;

  // 4th Equation/Row
  float m = 3*pow(t2,2);
  float n = 2*t2;
  float o = 1;
  float p = 0;
  float t = v2;

  float r1 = -b*g*l*t+b*g*p*s+b*h*k*t-b*h*o*s-b*k*p*r+b*l*o*r+c*f*l*t-c*f*p*s-c*h*j*t+c*h*n*s+c*j*p*r\
             -c*l*n*r-d*f*k*t+d*f*o*s+d*j*g*t-d*g*n*s-d*j*o*r+d*k*n*r+f*k*p*q-f*l*o*q-g*j*p*q+g*l*n*q+h*j*o*q-h*k*n*q;
  
  float r2 = a*g*l*t-a*g*p*s-a*h*k*t+a*h*o*s+a*k*p*r-a*l*o*r-c*e*l*t+c*e*p*s+c*h*i*t-c*h*m*s-c*i*p*r+c*l*m*r\
             +d*e*k*t-d*e*o*s-d*g*i*t+d*g*m*s+d*i*o*r-d*k*m*r-e*k*p*q+e*l*o*q+g*i*p*q-g*l*m*q-h*i*o*q+h*k*m*q;
  
  float r3 = -a*f*l*t+a*f*p*s+a*h*j*t-a*h*n*s-a*j*p*r+a*l*n*r+b*e*l*t-b*e*p*s-b*h*i*t+b*h*m*s+b*i*p*r-b*l*m*r\
             -d*e*j*t+d*e*n*s+d*f*i*t-d*f*m*s-d*i*n*r+d*j*m*r+e*j*p*q-e*l*n*q-f*i*p*q+f*l*m*q+h*i*n*q-h*j*m*q;
  
  float r4 = a*f*k*t-a*f*o*s-a*g*j*t+a*g*n*s+a*j*o*r-a*k*n*r-b*e*k*t+b*e*o*s+b*g*i*t-b*g*m*s-b*i*o*r+b*k*m*r+c*e*j*t\
             -c*e*n*s-c*f*i*t+c*f*m*s+c*i*n*r-c*j*m*r-e*j*o*q+e*k*n*q+f*i*o*q-f*k*m*q-g*i*n*q+g*j*m*q;
  
  float den = a*f*k*p-a*f*l*o-a*g*j*p+a*g*l*n+a*h*j*o-a*h*k*n-b*e*k*p+b*e*l*o+b*g*i*p-b*g*l*m-b*h*i*o+b*h*k*m\
              +c*e*j*p-c*e*l*n-c*f*i*p+c*f*l*m+c*h*i*n-c*h*j*m-d*e*j*o+d*e*k*n+d*f*i*o-d*f*k*m-d*g*i*n+d*g*j*m;

  float c1 = r1/den;
  float c2 = r2/den;
  float c3 = r3/den;
  float c4 = r4/den;

  coeff[0] = c1;
  coeff[1] = c2;
  coeff[2] = c3;
  coeff[3] = c4;
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

