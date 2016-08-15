/*  Arduino Library for the PCA9685 16-Channel PWM Driver Module.
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

    Created by Kasper Skårhøj, August 3rd, 2012.
    Forked by Vitska, June 18th, 2016.
    Forked by NachtRaveVL, July 29th, 2016.

    PCA9685-Arduino - Version 1.2.3
*/

#ifndef PCA9685_H
#define PCA9685_H

// Library Setup

// Uncomment this define if using the software i2c library (min 4MHz+ processor required)
//#define PCA9685_USE_SOFTWARE_I2C    1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define if wanting to exclude extended functionality from compilation
//#define PCA9685_EXCLUDE_EXT_FUNC    1

// Uncomment this define if wanting to exclude ServoEvaluator assistant from compilation
//#define PCA9685_EXCLUDE_SERVO_EVAL  1

// Uncomment this define to enable debug output
//#define PCA9685_DEBUG_OUTPUT        1

// Servo Control Note
// Many 180 degree controlled digital servos run on a 20ms pulse width (50Hz update
// frequency) based duty cycle, and do not utilize the entire pulse width for their
// -90/+90 degree control. Typically, 2.5% of the 20ms pulse width (0.5ms) is considered
// -90 degrees, and 12.5% of the 20ms pulse width (2.5ms) is considered +90 degrees. This
// roughly translates to raw PCA9685 PWM values of 102 and 512 (out of the 4096 value
// range) for -90 to +90 degree control, but may need to be adjusted to fit your specific
// servo (e.g. some I've tested run ~130 to ~525 for their -90/+90 degree control). Also
// be aware that driving some servos past their -90/+90 degrees of movement can cause a
// little plastic limiter pin to break off and get stuck inside of the gearing, which
// could potentially cause the servo to become jammed. See the PCA9685_ServoEvaluator
// class to assist with calculating PWM values from Servo angle values.

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#ifndef PCA9685_USE_SOFTWARE_I2C
#include <Wire.h>
#endif

#define PCA9685_MODE_INVRT          (byte)0x10  // Inverts polarity of channel output signal
#define PCA9685_MODE_OUTPUT_ONACK   (byte)0x08  // Channel update happens upon ACK (post-set) rather than on STOP (endTransmission)
#define PCA9685_MODE_OUTPUT_TPOLE   (byte)0x04  // Use a totem-pole (push-pull) style output, typical for boards using this chipset
#define PCA9685_MODE_OUTNE_HIGHZ    (byte)0x02  // For active low output enable, sets channel output to high-impedance state
#define PCA9685_MODE_OUTNE_LOW      (byte)0x01  // Similarly, sets channel output to high if in totem-pole mode, otherwise high-impedance state

#define PCA9685_MIN_CHANNEL         0
#define PCA9685_MAX_CHANNEL         15
#define PCA9685_CHANNEL_COUNT       16

typedef enum {
    PCA9685_PhaseBalancer_None = -1,    // Disables phase balancing, all high phase areas start at begining of cycle
    PCA9685_PhaseBalancer_Weaved = 0,   // Balances first few outputs better, steps away from previous shorten towards last output
    PCA9685_PhaseBalancer_Linear,       // Balances all outputs linearly, 256 steps away from previous output
    PCA9685_PhaseBalancer_Count
} PCA9685_PhaseBalancer;

class PCA9685 {
public:
#ifndef PCA9685_USE_SOFTWARE_I2C
    // May use a different Wire instance than Wire. Some chipsets, such as Due/Zero/etc.,
    // have a Wire1 class instance that uses the SDA1/SCL1 lines instead.
    // Supported i2c baud rates are 100kHz, 400kHz, and 1000kHz.
    PCA9685(TwoWire& i2cWire = Wire, PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_Weaved);
#else
    // Minimum supported baud rate is 100kHz, which means minimum supported processor
    // speed is 4MHz+ while running i2c standard mode. For 400kHz baud rate, minimum
    // supported processor speed is 16MHz+ while running i2c fast mode.
    PCA9685(PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_Weaved);
#endif

    // Should be called only once in setup(), before any init()'s, but after Wire.begin().
    // Only should be called once on any Wire instance to do a software reset, which
    // will affect all devices on that line. This helps when you're constantly rebuilding
    // and reuploading to ensure all the devices on that line are reset properly.
    void resetDevices();

    // The i2c address here is the value of the A0, A1, A2, A3, A4 and A5 pins ONLY,
    // as the class takes care of its internal base address. i2cAddress should be a
    // value between 0 and 61, since only 62 boards can be addressed.
    void init(byte i2cAddress = 0, byte mode = PCA9685_MODE_OUTPUT_ONACK | PCA9685_MODE_OUTPUT_TPOLE);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
    // Used when instance talks through to AllCall/Sub1-Sub3 instances as a proxy
    void initAsProxyAddresser(byte i2cAddress = 0xE0);
#endif

    // Min: 24Hz, Max: 1526Hz, Default: 200Hz (resolution widens as Hz goes higher)
    void setPWMFrequency(float pwmFrequency);

    // Turns channel either full on or full off
    void setChannelOn(int channel);
    void setChannelOff(int channel);

    // PWM amounts 0 - 4096, 0 full off, 4096 full on
    void setChannelPWM(int channel, word pwmAmount);
    // Max of 7 channels can be batched at once
    void setChannelsPWM(int startChannel, int count, const word *pwmAmounts);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
    // Sets all channels, but won't distribute phases
    void setAllChannelsPWM(word pwmAmount);

    // Returns PWM amounts 0 - 4096, 0 full off, 4096 full on
    word getChannelPWM(int channel);

    // Enables multiple talk-through paths via i2c bus (lsb/bit0 must stay 0)
    // To use, create a new class instance using initAsSubAddressed() with said address
    void enableAllCallAddress(byte i2cAddress = 0xE0);
    void enableSub1Address(byte i2cAddress = 0xE2);
    void enableSub2Address(byte i2cAddress = 0xE4);
    void enableSub3Address(byte i2cAddress = 0xE8);
    void disableAllCallAddress();
    void disableSub1Address();
    void disableSub2Address();
    void disableSub3Address();

    // Allows external clock line to be utilized (once enabled cannot be disabled)
    void enableExtClockLine();
#endif

private:
#ifndef PCA9685_USE_SOFTWARE_I2C
    TwoWire *_i2cWire;      // Wire class instance to use
#endif
    byte _i2cAddress;       // Module's i2c address
    PCA9685_PhaseBalancer _phaseBalancer; // Phase balancer scheme to distribute load
    bool _isProxyAddresser; // Instance is a proxy for sub addressing (disables certain functionality)

    void getPhaseCycle(int channel, word pwmAmount, word *phaseBegin, word *phaseEnd);

    void writeChannelBegin(int channel);
    void writeChannelEnd();
    void writeChannelPWM(word phaseBegin, word phaseEnd);

    void writeRegister(byte regAddress, byte value);
    byte readRegister(byte regAddress);

#ifdef PCA9685_USE_SOFTWARE_I2C
    uint8_t _readBytes;
#endif
    void i2cWire_beginTransmission(uint8_t);
    uint8_t i2cWire_endTransmission(void);
    uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
    size_t i2cWire_write(uint8_t);
    int i2cWire_read(void);
};

#ifndef PCA9685_EXCLUDE_SERVO_EVAL

// Class to assist with calculating Servo PWM values from angle values
class PCA9685_ServoEvaluator {
public:
    // Uses a linear interpolation method to quickly compute PWM output value. Uses
    // default values of 2.5% and 12.5% of phase length for -90/+90.
    PCA9685_ServoEvaluator(word n90PWMAmount = 102, word p90PWMAmount = 512);

    // Uses a cubic spline to interpolate due to an offsetted zero angle that isn't
    // exactly between -90/+90. This takes more time to compute, but gives a more
    // accurate PWM output value along the entire range.
    PCA9685_ServoEvaluator(word n90PWMAmount, word zeroPWMAmount, word p90PWMAmount);

    ~PCA9685_ServoEvaluator();

    // Returns the PWM value to use given the angle (-90 to +90)
    word pwmForAngle(float angle);

private:
    float *_coeff;      // a,b,c,d coefficient values
    bool _isCSpline;    // Cubic spline tracking, for _coeff length
};

#endif

#endif 
