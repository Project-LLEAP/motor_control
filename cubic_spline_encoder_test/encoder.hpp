#ifndef ENCODER_HPP
#def ENCODER_HPP

uint16_t readEncoderPosition14Bit(void);
float encoderReadingToDeg(uint16_t position);
float readEncoderPositionDeg(void);

#endif  // ENCODER_HPP
