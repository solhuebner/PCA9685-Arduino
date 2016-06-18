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
bool PCA9685::init() {

  delay(1);
  writeRegister(PCA9685_MODE1, (byte)0x01); // reset the device

  delay(1);
  bool isOnline;
  if (readRegister(PCA9685_MODE1)==0x01)  {
    isOnline = true;
  } else {
    isOnline = false;
  }
  writeRegister(PCA9685_MODE1, (byte)B10100000);  // set up for auto increment
  writeRegister(PCA9685_MODE2, (byte)0x10); // set to output
  
  return isOnline;
}

void PCA9685::on(int nr) {
  writeChannel(nr, PCA9685_PWM_FULL, 0);
}

void PCA9685::off(int nr) {
  writeChannel(nr, 0, PCA9685_PWM_FULL);
}

void PCA9685::setChannel(int nr, word amount) {    // Amount from 0-100 (off-on)
  word onval = 0, offval = 0;
  if (amount==PCA9685_CH_OFF){
    onval = 0;
    offval = PCA9685_PWM_FULL;
  }
  else if (amount>=PCA9685_CH_ON){
    onval = PCA9685_PWM_FULL;
    offval = 0;
  } 
  else {
    //int randNumber = (int)random(4096); // Randomize the phaseshift to distribute load. Good idea? Hope so.
    word phase = (4096 / 16) * nr;
    onval = phase;
    offval = (amount+phase) & 0xFFF;
  }
  
  writeChannel(nr, onval, offval);
}

void PCA9685::setChannel(byte startch, byte count, word* amount) {    // Amount from 0-100 (off-on)
  Wire.beginTransmission(_i2cAddress);
  Wire.write(PCA9685_LED0 + 4*startch);
  for(byte i = 0; i < count; i ++){
    byte nr = startch+i;
    word onval = 0, offval = 0;
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
      word phase = (4096 / 16) * nr;
      onval = phase;
      offval = (amount[i]+phase) & 0xFFF;
    }
    
    Wire.write(lowByte(onval));
    Wire.write(highByte(onval));
    Wire.write(lowByte(offval));
    Wire.write(highByte(offval));
  }
  Wire.endTransmission();
}

void PCA9685::writeChannel(int ledNumber, word LED_ON, word LED_OFF) {  // LED_ON and LED_OFF are 12bit values (0-4095); ledNumber is 0-15
  if (ledNumber >=0 && ledNumber <= 15) {
    
    Wire.beginTransmission(_i2cAddress);
    Wire.write(PCA9685_LED0 + 4*ledNumber);

    Wire.write(lowByte(LED_ON));
    Wire.write(highByte(LED_ON));
    Wire.write(lowByte(LED_OFF));
    Wire.write(highByte(LED_OFF));
    
    Wire.endTransmission();
  }
}


//PRIVATE
void PCA9685::writeRegister(int regAddress, byte data) {
  Wire.beginTransmission(_i2cAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

word PCA9685::readRegister(int regAddress) {
  word returnword = 0x00;
  Wire.beginTransmission(_i2cAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)_i2cAddress, 1);
    
//    int c=0;
  while (Wire.available()) {
    returnword |= Wire.read(); 
  }
    
  return returnword;
}