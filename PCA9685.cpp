/*  PCA9685 LED library for Arduino
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
#include "PCA9685.h"

void PCA9685::begin(int i2cAddress) {
  _i2cAddress = PCA9685_I2C_BASE_ADDRESS | (i2cAddress & B00111111);
}

void PCA9685::init(byte mode) {
  //Reset SWRST
  Wire.beginTransmission(0);
  Wire.write((byte)0x06);
  Wire.endTransmission();
  //Init
  Wire.beginTransmission(_i2cAddress);
  Wire.write(PCA9685_MODE1);
  Wire.write(0b00100000); //Autoincrement
  Wire.write(mode);
  Wire.endTransmission();
}

void PCA9685::setPwmFrequency(int hz){
  uint32_t val = 6104L / (uint32_t)hz;
  if(val > 255)val = 255;
  if(val < 3)val = 3;
  Wire.beginTransmission(_i2cAddress);
  Wire.write(PCA9685_PRE_SCALE);
  Wire.write((byte)val);
  Wire.endTransmission();
}

void PCA9685::on(int nr) {
  writeChBegin(nr);
  writeChannel(PCA9685_PWM_FULL, 0);
  writeEnd();
}

void PCA9685::off(int nr) {
  writeChBegin(nr);
  writeChannel(0, PCA9685_PWM_FULL);
  writeEnd();
}

void PCA9685::setChannel(byte nr, word amount) {    // Amount from 0-100 (off-on)
  writeChBegin(nr);
  if (amount==PCA9685_CH_OFF){
    writeChannel(0, PCA9685_PWM_FULL);
  }
  else if (amount>=PCA9685_CH_ON){
    writeChannel(PCA9685_PWM_FULL, 0);
  } 
  else {
    //int randNumber = (int)random(4096); // Randomize the phaseshift to distribute load. Good idea? Hope so.
    word phase = (4096 / 16) * nr;
    writeChannel(phase, (amount+phase) & 0xFFF);
  }
  writeEnd();
}

// IN:avr/libraries/Wire.h 
//    avr/libraries/utility/twi.h
//  BUFFER_LENGTH should be increased to 64 + 2 at least, default is 32 so only 7 lines could be written in one transaction
void PCA9685::setChannel(byte startch, byte count, const word* amount) {    // Amount from 0-100 (off-on)
  writeChBegin(startch);
  word onval = 0, offval = 0;
  for(byte i = 0; i < count; i++){
    if (amount[i]==PCA9685_CH_OFF){
      onval = 0;
      offval = PCA9685_PWM_FULL;
    }
    else if (amount[i]>=PCA9685_CH_ON){
      onval = PCA9685_PWM_FULL;
      offval = 0;
    } 
    else {
      //int randNumber = (int)random(4096); // Randomize the phaseshift to distribute load. Good idea? Hope so.
      byte nr = startch+i;
      word phase = (4096 / 16) * nr;
      onval = phase;
      offval = (amount[i]+phase) & 0xFFF;
    }
    writeChannel(onval, offval);
  }
  writeEnd();
}

void PCA9685::writeChBegin(int nr) {  // LED_ON and LED_OFF are 12bit values (0-4095); ledNumber is 0-15
    byte addr = PCA9685_LED0 + 4*nr;
    Wire.beginTransmission(_i2cAddress);
    Wire.write(addr);
}

void PCA9685::writeEnd() {  // LED_ON and LED_OFF are 12bit values (0-4095); ledNumber is 0-15
    Wire.endTransmission();
}

void PCA9685::writeChannel(word LED_ON, word LED_OFF) {  // LED_ON and LED_OFF are 12bit values (0-4095); ledNumber is 0-15
    Wire.write(lowByte(LED_ON));
    Wire.write(highByte(LED_ON));
    Wire.write(lowByte(LED_OFF));
    Wire.write(highByte(LED_OFF));
}

