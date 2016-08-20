# PCA9685-Arduino
Arduino Library for the PCA9685 16-Channel PWM Driver Module.

**PCA9685-Arduino - Version 1.2.6**

Library to control a PCA9685 16-channel PWM driver module from an Arduino board.  
Licensed under the copy-left GNU GPL v3 license.

Created by Kasper Skårhøj, August 3rd, 2012.  
Forked by Vitska, June 18th, 2016.  
Forked by NachtRaveVL, July 29th, 2016.

## Library Setup

There are several defines inside of the library's header file that allows for more fine-tuned control.

```Arduino
// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define PCA9685_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define if wanting to exclude extended functionality from compilation.
//#define PCA9685_EXCLUDE_EXT_FUNC        1

// Uncomment this define if wanting to exclude ServoEvaluator assistant from compilation.
//#define PCA9685_EXCLUDE_SERVO_EVAL      1

// Uncomment this define to enable debug output.
//#define PCA9685_ENABLE_DEBUG_OUTPUT     1
```

## Servo Control Note

Many 180 degree controlled digital servos run on a 20ms pulse width (50Hz update frequency) based duty cycle, and do not utilize the entire pulse width for their -90/+90 degree control. Typically, 2.5% of the 20ms pulse width (0.5ms) is considered -90 degrees, and 12.5% of the 20ms pulse width (2.5ms) is considered +90 degrees. This roughly translates to raw PCA9685 PWM values of 102 and 512 (out of the 4096 value range) for -90 to +90 degree control, but may need to be adjusted to fit your specific servo (e.g. some I've tested run ~130 to ~525 for their -90/+90 degree control). Also be aware that driving some servos past their -90/+90 degrees of movement can cause a little plastic limiter pin to break off and get stuck inside of the gearing, which could potentially cause the servo to become jammed. See the PCA9685_ServoEvaluator class to assist with calculating PWM values from Servo angle values.

## Example Usage

Below are several examples of library usage.

### Simple Example
```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmDriver;

void setup() {
    Serial.begin(115200);
    Wire.begin();                   // Wire must be started first
    Wire.setClock(400000);          // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmDriver.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmDriver.init(B000000);        // Address pins A5-A0 set to B000000
    pwmDriver.setPWMFrequency(100); // Default is 200Hz, supports 24Hz to 1526Hz


    pwmDriver.setChannelPWM(0, 128 << 4); // Set PWM to 128/255, but in 4096 land

    Serial.println(pwmDriver.getChannelPWM(0)); // Should output 2048, which is 128 << 4
}

```

### Batching Example
```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmDriver;

void setup() {
    Wire.begin();                   // Wire must be started first
    Wire.setClock(400000);          // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmDriver.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmDriver.init(B010101);        // Address pins A5-A0 set to B010101
    pwmDriver.setPWMFrequency(500); // Default is 200Hz, supports 24Hz to 1526Hz

    randomSeed(analogRead(0));      // Use white noise for our randomness
}

void loop() {
    word pwms[12];
    pwms[0] = random(0, 4096);
    pwms[1] = random(0, 4096);
    pwms[2] = random(0, 4096);
    pwms[3] = random(0, 4096);
    pwms[4] = random(0, 4096);
    pwms[5] = random(0, 4096);
    pwms[6] = random(0, 4096);
    pwms[7] = random(0, 4096);
    pwms[8] = random(0, 4096);
    pwms[9] = random(0, 4096);
    pwms[10] = random(0, 4096);
    pwms[11] = random(0, 4096);
    pwmDriver.setChannelsPWM(0, 12, pwms);
    delay(500);

    // Note that only 7 channels can be written in one i2c transaction due to a
    // BUFFER_LENGTH limit of 32, so 12 channels will take two i2c transactions.
}

```

### Multi-Device Proxy Example
```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmDriver1;
PCA9685 pwmDriver2;

PCA9685 pwmDriverAll;               // Not a real device, will act as a proxy to pwmDriver1 and pwmDriver2

void setup() {
    Serial.begin(115200);
    Wire.begin();                   // Wire must be started first
    Wire.setClock(400000);          // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmDriver1.resetDevices();      // Software resets all PCA9685 devices on Wire line (including pwmDriver2 in this case)

    pwmDriver1.init(B000000);       // Address pins A5-A0 set to B000000
    pwmDriver2.init(B000001);       // Address pins A5-A0 set to B000001


    pwmDriver1.setChannelOff(0);    // Turn channel 0 off
    pwmDriver2.setChannelOff(0);    // On both

    pwmDriver1.enableAllCallAddress(); // Default address of 0xE0
    pwmDriver2.enableAllCallAddress(); // Same default address

    pwmDriverAll.initAsProxyAddresser(); // Same default address of 0x0E as used in enable above

    pwmDriverAll.setChannelPWM(0, 4096); // Enables full on on both pwmDriver1 and pwmDriver2

    Serial.println(pwmDriver1.getChannelPWM(0)); // Should output 4096
    Serial.println(pwmDriver2.getChannelPWM(0)); // Should also output 4096

    // Note: Various parts of functionality of the proxy class instance actually
    // are disabled, typically anything that involves a read command being issued.
}

```

### Servo Evaluator Example
```Arduino
#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmDriver;

// Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°
PCA9685_ServoEvaluator pwmServo1;

// Testing our second servo has found that -90° sits at 128, 0° at 324, and +90° at 526.
// Since 324 isn't precisely in the middle, a cubic spline will be used to smoothly
// interpolate PWM values, which will account for said discrepancy. Additionally, since
// 324 is closer to 128 than 526, there is less resolution in the -90° to 0° range, and
// more in the 0° to +90° range.
PCA9685_ServoEvaluator pwmServo2(128,324,526);

void setup() {
    Serial.begin(115200);
    Wire.begin();                   // Wire must be started first
    Wire.setClock(400000);          // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmDriver.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmDriver.init(B000000);        // Address pins A5-A0 set to B000000
    pwmDriver.setPWMFrequency(50);  // 50Hz provides 20ms standard servo phase length
    

    pwmDriver.setChannelPWM(0, pwmServo1.pwmForAngle(-90));
    Serial.println(pwmDriver.getChannelPWM(0)); // Should output 102 for -90°

    // Showing linearity for midpoint, 205 away from both -90° and 90°
    Serial.println(pwmServo1.pwmForAngle(0));   // Should output 307 for 0°

    pwmDriver.setChannelPWM(0, pwmServo1.pwmForAngle(90));
    Serial.println(pwmDriver.getChannelPWM(0)); // Should output 512 for +90°


    pwmDriver.setChannelPWM(1, pwmServo2.pwmForAngle(-90));
    Serial.println(pwmDriver.getChannelPWM(1)); // Should output 128 for -90°
    
    // Showing less resolution in the -90° to 0° range
    Serial.println(pwmServo2.pwmForAngle(-45)); // Should output 225 for -45°, 97 away from -90°

    pwmDriver.setChannelPWM(1, pwmServo2.pwmForAngle(0));
    Serial.println(pwmDriver.getChannelPWM(1)); // Should output 324 for 0°

    // Showing more resolution in the 0° to +90° range
    Serial.println(pwmServo2.pwmForAngle(45));  // Should output 424 for +45°, 102 away from +90°

    pwmDriver.setChannelPWM(1, pwmServo2.pwmForAngle(90));
    Serial.println(pwmDriver.getChannelPWM(1)); // Should output 526 for +90°
}

```
