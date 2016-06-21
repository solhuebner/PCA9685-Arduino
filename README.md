# PCA9685-Arduino-Library
Arduino Library for the PCA9685 LED Driver chip

## Example
```Arduino
#include "PCA9685.h"

PCA9685 pwmDriver; 

void setup() {
  Wire.begin();           // Wire must be started!
  Wire.setClock(400000L);
  pwmDriver.begin(B000000);  // Address pins A5-A0 set to B111000
  pwmDriver.init();
  pwmDriver.setPwmFrequency(1500);
}

void loop() {
  word pwms[PCA9685_CHANNEL_COUNT];
  pwms[0] = PCA9685_CH_OFF;
  pwms[1] = PCA9685_CH_OFF;
  pwms[2] = PCA9685_CH_OFF;
  pwms[3] = PCA9685_CH_OFF;
  pwms[4] = PCA9685_CH_OFF;
  pwms[5] = PCA9685_CH_OFF;
  pwms[6] = PCA9685_CH_OFF;
  pwms[7] = PCA9685_CH_OFF;
  pwms[8] = PCA9685_CH_OFF;
  pwms[9] = PCA9685_CH_OFF;
  pwms[10] = PCA9685_CH_OFF;
  pwms[11] = PCA9685_CH_OFF;
  pwms[12] = PCA9685_CH_OFF;
  pwms[13] = PCA9685_CH_OFF;
  pwms[14] = PCA9685_CH_OFF;
  pwms[15] = PCA9685_CH_OFF;
  pwmDriver.setChannel(PCA9685_MIN_CHANNEL, PCA9685_CHANNEL_COUNT, pwms);
  delay(250);
  
  pwms[0] = PCA9685_CH_ON;
  pwms[1] = PCA9685_CH_ON;
  pwms[2] = PCA9685_CH_ON;
  pwms[3] = PCA9685_CH_ON;
  pwms[4] = PCA9685_CH_ON;
  pwms[5] = PCA9685_CH_ON;
  pwms[6] = PCA9685_CH_ON;
  pwms[7] = PCA9685_CH_ON;
  pwms[8] = PCA9685_CH_ON;
  pwms[9] = PCA9685_CH_ON;
  pwms[10] = PCA9685_CH_ON;
  pwms[11] = PCA9685_CH_ON;
  pwms[12] = PCA9685_CH_ON;
  pwms[13] = PCA9685_CH_ON;
  pwms[14] = PCA9685_CH_ON;
  pwms[15] = PCA9685_CH_ON;
  pwmDriver.setChannel(PCA9685_MIN_CHANNEL, PCA9685_CHANNEL_COUNT, pwms);
  delay(250);
}

```
