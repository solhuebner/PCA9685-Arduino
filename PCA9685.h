/*  PCA9685 library for Arduino
    Copyright (C) 2012 Kasper Skårhøj    <kasperskaarhoj@gmail.com> 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef PCA9685_H
#define PCA9685_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

/**
  Version 1.0.0
  (Semantic Versioning)
**/
 
//Register defines from data sheet
#define PCA9685_MODE1 0x00 // location for Mode1 register address
#define PCA9685_MODE2 0x01 // location for Mode2 reigster address
#define PCA9685_LED0 0x06 // location for start of LED0 registers

#define PCA9685_MODE_INVRT 0x10
#define PCA9685_MODE_OUTPUT_ON_ACK 0x08
#define PCA9685_MODE_OUTPUT_POLE 0x04

#define PCA9685_CH_ON 4096 
#define PCA9685_CH_OFF 0
#define PCA9685_PWM_FULL 0x1000 

#define PCA9685_I2C_BASE_ADDRESS B1000000

class PCA9685
{
  public:
    //NB the i2c address here is the value of the A0, A1, A2, A3, A4 and A5 pins ONLY
    //as the class takes care of its internal base address.
    //so i2cAddress should be between 0 and 63
	PCA9685(){}
    void begin(int i2cAddress);
    bool init(byte mode = PCA9685_MODE_INVRT | PCA9685_MODE_OUTPUT_ON_ACK | PCA9685_MODE_OUTPUT_POLE);

  void on(int nr);
  void off(int nr);
  // 0: instant OFF(0) 
  // 4096: instant ON(1) 
  // 1 - 4095: duty sicle
  void setChannel(int nr, uint16_t amount);
  void setChannel(byte startch, byte count, word* amount);
  
  private:
  void writeChannel(int nr, word outputStart, word outputEnd);
  void writeRegister(int regaddress, byte val);
  word readRegister(int regAddress);
  // Our actual i2c address:
  byte _i2cAddress;
};
#endif 