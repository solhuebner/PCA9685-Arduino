/*  Arduino Library for the PCA9685 16-Channel PWM Driver Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    Copyright (C) 2012 Kasper Skårhøj       <kasperskaarhoj@gmail.com>

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

    PCA9685-Arduino - Version 1.2.15
*/

#ifndef PCA9685_H
#define PCA9685_H

// Library Setup

// NOTE: It is recommended to avoid editing library files directly and instead use custom
// build flags. While most custom build systems support such, the Arduino IDE does not.
// Be aware that editing this file directly will affect all projects using this library.

// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define PCA9685_ENABLE_SOFTWARE_I2C             // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define if wanting to exclude extended functionality from compilation.
//#define PCA9685_EXCLUDE_EXT_FUNC

// Uncomment this define if wanting to exclude ServoEvaluator assistant from compilation.
//#define PCA9685_EXCLUDE_SERVO_EVAL

// Uncomment this define to swap PWM low(begin)/high(end) phase values in register reads/writes (needed for some chip manufacturers).
//#define PCA9685_SWAP_PWM_BEG_END_REGS

// Uncomment this define to enable debug output.
//#define PCA9685_ENABLE_DEBUG_OUTPUT

// Hookup Callout: Servo Control
// -PLEASE READ-
// Many 180 degree controlled digital servos run on a 20ms pulse width (50Hz update
// frequency) based duty cycle, and do not utilize the entire pulse width for their
// -90/+90 degree control. Typically, 2.5% of the 20ms pulse width (0.5ms) is considered
// -90 degrees, and 12.5% of the 20ms pulse width (2.5ms) is considered +90 degrees.
// This roughly translates to raw PCA9685 PWM values of 102 and 512 (out of the 4096
// value range) for -90 to +90 degree control, but may need to be adjusted to fit your
// specific servo (e.g. some I've tested run ~130 to ~525 for their -90/+90 degree
// control).
//
// -ALSO-
// Please be aware that driving some servos past their -90/+90 degrees of movement can
// cause a little plastic limiter pin to break off and get stuck inside of the gearing,
// which could potentially cause the servo to become jammed and no longer function.
//
// See the PCA9685_ServoEvaluator class to assist with calculating PWM values from Servo
// angle values, if you desire that level of fine tuning.

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#ifndef PCA9685_ENABLE_SOFTWARE_I2C
#include <Wire.h>
#if BUFFER_LENGTH
#define PCA9685_I2C_BUFFER_LENGTH   BUFFER_LENGTH
#elif I2C_BUFFER_LENGTH
#define PCA9685_I2C_BUFFER_LENGTH   I2C_BUFFER_LENGTH
#else
#warning "i2c buffer length not defined - using default value of 32, which may not be correct for your microcontroller. Check Wire.h (or similar) for your hardware and manually define BUFFER_LENGTH or I2C_BUFFER_LENGTH to remove this warning."
#define PCA9685_I2C_BUFFER_LENGTH   32
#endif // /if BUFFER_LENGTH
#else
#define PCA9685_USE_SOFTWARE_I2C
#endif // /ifndef PCA9685_ENABLE_SOFTWARE_I2C

// Default proxy addresser i2c addresses
#define PCA9685_I2C_DEF_ALLCALL_PROXYADR    (byte)0xE0      // Default AllCall i2c proxy address
#define PCA9685_I2C_DEF_SUB1_PROXYADR       (byte)0xE2      // Default Sub1 i2c proxy address
#define PCA9685_I2C_DEF_SUB2_PROXYADR       (byte)0xE4      // Default Sub2 i2c proxy address
#define PCA9685_I2C_DEF_SUB3_PROXYADR       (byte)0xE8      // Default Sub3 i2c proxy address


// Output driver control mode (see datasheet Table 12 and Fig 13, 14, and 15 concerning correct
// usage of OUTDRV).
enum PCA9685_OutputDriverMode {
    PCA9685_OutputDriverMode_OpenDrain,         // Module outputs in an open-drain style structure, without an external driver
    PCA9685_OutputDriverMode_TotemPole,         // Module outputs in a totem-pole style structure, with an external driver (default)

    PCA9685_OutputDriverMode_Count,             // Internal use only
    PCA9685_OutputDriverMode_Undefined = -1     // Internal use only
};
// NOTE: Totem-pole mode requires an external driver to be used (most breakouts support
// this, some don't). However, from datasheet Table 6. subnote [1]: "Some newer LEDs
// include integrated Zener diodes to limit voltage transients, reduce EMI, and protect
// the LEDs, and these -MUST BE- driven only in the open-drain mode to prevent over-
// heating the IC."

// Output-enabled/active-low-OE-pin=LOW driver output mode (see datasheet Table 12 and
// Fig 13, 14, and 15 concerning correct usage of INVRT).
enum PCA9685_OutputEnabledMode {
    PCA9685_OutputEnabledMode_Normal,           // When OE is enabled/LOW, channel output uses normal output polarity (default)
    PCA9685_OutputEnabledMode_Inverted,         // When OE is enabled/LOW, channel output uses inverted output polarity (only available in totem-pole mode)

    PCA9685_OutputEnabledMode_Count,            // Internal use only
    PCA9685_OutputEnabledMode_Undefined = -1    // Internal use only
};
// NOTE: Polarity inversion should be set according to if an external N-type driver
// (should use INVRT) or P-type driver (should not use INVRT) is used.

// Output-not-enabled/active-low-OE-pin=HIGH driver output mode (see datasheet Section
// 7.4 concerning correct usage of OUTNE).
enum PCA9685_OutputDisabledMode {
    PCA9685_OutputDisabledMode_Low,             // When OE is disabled/HIGH, channels output a LOW signal (default)
    PCA9685_OutputDisabledMode_High,            // When OE is disabled/HIGH, channels output a HIGH signal (only available in totem-pole mode)
    PCA9685_OutputDisabledMode_Floating,        // When OE is disabled/HIGH, channel outputs go into a high-impediance/floating state (aka high-Z), which may be further refined via external pull-up/pull-down resistors

    PCA9685_OutputDisabledMode_Count,           // Internal use only
    PCA9685_OutputDisabledMode_Undefined = -1   // Internal use only
};
// NOTE: Active-low-OE pin is typically used to synchronize multiple PCA9685 devices
// together, but can also be used as an external dimming control signal.

// Channel update strategy used when multiple channels are being updated in batch.
enum PCA9685_ChannelUpdateMode {
    PCA9685_ChannelUpdateMode_AfterStop,        // Channel updates commit after full-transmission STOP signal (default)
    PCA9685_ChannelUpdateMode_AfterAck,         // Channel updates commit after individual channel update ACK signal

    PCA9685_ChannelUpdateMode_Count,            // Internal use only
    PCA9685_ChannelUpdateMode_Undefined = -1    // Internal use only
};

// Software-based phase balancing scheme.
enum PCA9685_PhaseBalancer {
    PCA9685_PhaseBalancer_None,                 // Disables software-based phase balancing, relying on installed hardware to handle current sinkage (ensure 10v 1000μF capacitor is installed on breakout/circuit)  (default)
    PCA9685_PhaseBalancer_Linear,               // Uses linear software-based phase balancing, with each channel being a preset 256 steps away from previous channel (may cause flickering on PWM changes)
    PCA9685_PhaseBalancer_Weaved,               // Uses weaved software-based phase balancing, with each channel being positioned in a preset weaved distribution that favors fewer/lower-indexed channels (may cause flickering on PWM changes)
    PCA9685_PhaseBalancer_Dynamic,              // Uses dynamic software-based phase balancing, with each modified channel entering an in-use pool that recalculates an automatic linear distribution based on total channels modified/in-use (may cause flickering on PWM changes)

    PCA9685_PhaseBalancer_Count,                // Internal use only
    PCA9685_PhaseBalancer_Undefined = -1        // Internal use only
};
// NOTE: Software-based phase balancing attempts to mitigate the situation whereby a
// large current sink can occur at the start of the PWM phase range, especially when
// multiple PWM channels are active. It does this by shifting the rising edge of each
// PWM duty cycle by some amount so that this current sink occurs over the entire phase
// range instead of all at once at the start of the phase range. Software-based phase
// balancing is only necessary in situations where there isn't a hardware-based solution
// present, such as when a proper capacitor is installed to handle current sinkage, as
// is installed in most, but not all, breakouts.


class PCA9685 {
public:
#ifndef PCA9685_USE_SOFTWARE_I2C
    // Library constructor. Typically called during class instantiation, before setup().
    // The i2c address should be the value of the A5-A0 pins, as the class handles the
    // module's base i2c address. It should be a value between 0 and 61, which gives a
    // maximum of 62 modules that can be addressed on the same i2c line.
    // Boards with more than one i2c line (e.g. Due/Zero/etc.) may use a different Wire
    // instance, such as Wire1 (which uses SDA1/SCL1 pins), Wire2 (SDA2/SCL2), etc.
    // Supported i2c clock speeds are 100kHz, 400kHz (default), and 1000kHz.
    PCA9685(byte i2cAddress = B000000, TwoWire& i2cWire = Wire, uint32_t i2cSpeed = 400000);

    // Convenience constructor for custom Wire instance. See main constructor.
    PCA9685(TwoWire& i2cWire, uint32_t i2cSpeed = 400000, byte i2cAddress = B000000);
#else
    // Library constructor. Typically called during class instantiation, before setup().
    // The i2c address should be the value of the A5-A0 pins, as the class handles the
    // module's base i2c address. It should be a value between 0 and 61, which gives a
    // maximum of 62 modules that can be addressed on the same i2c line.
    // Minimum supported i2c clock speed is 100kHz, which means minimum supported processor
    // speed is 4MHz+ while running i2c standard mode. For 400kHz i2c clock speed, minimum
    // supported processor speed is 16MHz+ while running i2c fast mode.
    PCA9685(byte i2cAddress = B000000);
#endif

    // Resets modules, also begins Wire instance. Typically called in setup(), before any
    // init()'s. Calling will perform a software reset on all PCA9685 devices on the Wire
    // instance, ensuring that all PCA9685 devices on that line are properly reset.
    void resetDevices();

    // Initializes module, also begins Wire instance. Typically called in setup().
    // See individual enums for more info.
    void init(PCA9685_OutputDriverMode driverMode = PCA9685_OutputDriverMode_TotemPole,
              PCA9685_OutputEnabledMode enabledMode = PCA9685_OutputEnabledMode_Normal,
              PCA9685_OutputDisabledMode disabledMode = PCA9685_OutputDisabledMode_Low,
              PCA9685_ChannelUpdateMode updateMode = PCA9685_ChannelUpdateMode_AfterStop,
              PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_None);

    // Convenience initializer for custom phase balancer. See main init method.
    void init(PCA9685_PhaseBalancer phaseBalancer,
              PCA9685_OutputDriverMode driverMode = PCA9685_OutputDriverMode_TotemPole,
              PCA9685_OutputEnabledMode enabledMode = PCA9685_OutputEnabledMode_Normal,
              PCA9685_OutputDisabledMode disabledMode = PCA9685_OutputDisabledMode_Low,
              PCA9685_ChannelUpdateMode updateMode = PCA9685_ChannelUpdateMode_AfterStop);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
    // Initializes module as a proxy addresser, also begins Wire instance. Typically
    // called in setup(). Used when instance talks through to AllCall/Sub1-Sub3 instances
    // as a proxy object. Using this method will disable any method that performs a read
    // or conflicts with certain states. Proxy addresser i2c addresses must be >= 0xE0,
    // with defaults provided via PCA9685_I2C_DEF_[ALLCALL|SUB[1-3]]_PROXYADR defines.
    void initAsProxyAddresser();
#endif

    // Mode accessors
    byte getI2CAddress();
    uint32_t getI2CSpeed();
    PCA9685_OutputDriverMode getOutputDriverMode();
    PCA9685_OutputEnabledMode getOutputEnabledMode();
    PCA9685_OutputDisabledMode getOutputDisabledMode();
    PCA9685_ChannelUpdateMode getChannelUpdateMode();
    PCA9685_PhaseBalancer getPhaseBalancer();

    // Min: 24Hz, Max: 1526Hz, Default: 200Hz (as Hz increases channel resolution widens, but raw pre-scaler value, as computed per datasheet, also becomes less affected inversly)
    void setPWMFrequency(float pwmFrequency);

    // Turns channel either full on or full off
    void setChannelOn(int channel);
    void setChannelOff(int channel);

    // PWM amounts 0 - 4096, 0 full off, 4096 full on
    void setChannelPWM(int channel, uint16_t pwmAmount);
    void setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
    // Sets all channels, but won't distribute phases
    void setAllChannelsPWM(uint16_t pwmAmount);

    // Returns PWM amounts 0 - 4096, 0 full off, 4096 full on
    uint16_t getChannelPWM(int channel);

    // Enables multiple talk-through paths via i2c bus (lsb/bit0 must stay 0). To use,
    // create a new proxy instance using initAsProxyAddresser() with proper proxy i2c
    // address >= 0xE0, and pass that instance's i2c address into desired method below.
    void enableAllCallAddress(byte i2cAddressAllCall = PCA9685_I2C_DEF_ALLCALL_PROXYADR);
    void enableSub1Address(byte i2cAddressSub1 = PCA9685_I2C_DEF_SUB1_PROXYADR);
    void enableSub2Address(byte i2cAddressSub2 = PCA9685_I2C_DEF_SUB2_PROXYADR);
    void enableSub3Address(byte i2cAddressSub3 = PCA9685_I2C_DEF_SUB3_PROXYADR);
    void disableAllCallAddress();
    void disableSub1Address();
    void disableSub2Address();
    void disableSub3Address();

    // Allows external clock line to be utilized (once enabled cannot be disabled)
    void enableExtClockLine();
#endif

    byte getLastI2CError();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    int getWireInterfaceNumber();
    void printModuleInfo();
    void checkForErrors();
#endif

protected:
    static bool _i2cBegan;                                  // Global i2c began flag
    byte _i2cAddress;                                       // Module's i2c address (default: B000000)
#ifndef PCA9685_USE_SOFTWARE_I2C
    TwoWire* _i2cWire;                                      // Wire class instance (unowned) (default: Wire)
    uint32_t _i2cSpeed;                                     // Module's i2c clock speed (default: 400000)
#endif
    PCA9685_OutputDriverMode _driverMode;                   // Output driver mode
    PCA9685_OutputEnabledMode _enabledMode;                 // OE enabled output mode
    PCA9685_OutputDisabledMode _disabledMode;               // OE disabled output mode
    PCA9685_ChannelUpdateMode _updateMode;                  // Channel update mode
    PCA9685_PhaseBalancer _phaseBalancer;                   // Phase balancer scheme
    bool _isProxyAddresser;                                 // Proxy addresser flag (disables certain functionality)
    byte _lastI2CError;                                     // Last module i2c error

    byte getMode2Value();
    void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);

    void writeChannelBegin(int channel);
    void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
    void writeChannelEnd();

    void writeRegister(byte regAddress, byte value);
    byte readRegister(byte regAddress);

#ifdef PCA9685_USE_SOFTWARE_I2C
    uint8_t _readBytes;
#endif
    void i2cWire_begin();
    void i2cWire_beginTransmission(uint8_t);
    uint8_t i2cWire_endTransmission(void);
    uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
    size_t i2cWire_write(uint8_t);
    uint8_t i2cWire_read(void);
};

#ifndef PCA9685_EXCLUDE_SERVO_EVAL

// Class to assist with calculating Servo PWM values from angle values
class PCA9685_ServoEvaluator {
public:
    // Uses a linear interpolation method to quickly compute PWM output value. Uses
    // default values of 2.5% and 12.5% of phase length for -90/+90.
    PCA9685_ServoEvaluator(uint16_t n90PWMAmount = 102, uint16_t p90PWMAmount = 512);

    // Uses a cubic spline to interpolate due to an offsetted zero angle that isn't
    // exactly between -90/+90. This takes more time to compute, but gives a more
    // accurate PWM output value along the entire range.
    PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount);

    ~PCA9685_ServoEvaluator();

    // Returns the PWM value to use given the angle (-90 to +90)
    uint16_t pwmForAngle(float angle);

private:
    float *_coeff;      // a,b,c,d coefficient values
    bool _isCSpline;    // Cubic spline tracking, for _coeff length
};

#endif

#endif 