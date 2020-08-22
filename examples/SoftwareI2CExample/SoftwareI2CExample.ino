// PCA9685-Arduino Software I2C Example
// In this example, we utilize the software I2C functionality for chips that do not have
// a hardware I2C bus. If one uncomments the line below inside the main header file (or
// defines it via custom build flag), software I2C mode for the library will be enabled.
//
// In PCA9685.h:
// // Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
// #define PCA9685_ENABLE_SOFTWARE_I2C             // http://playground.arduino.cc/Main/SoftwareI2CLibrary

#include "PCA9685.h"

#define SCL_PIN 2                       // Setup defines for SoftI2CMaster are written before library include
#define SCL_PORT PORTD 
#define SDA_PIN 0 
#define SDA_PORT PORTC 

#if F_CPU >= 16000000
#define I2C_FASTMODE 1                  // Running a 16MHz processor allows us to use i2c fast mode
#endif

#include "SoftI2CMaster.h"              // Include must come after setup defines (see library setup)

PCA9685 pwmController;                  // Library using default B000000 (A5-A0) i2c address

void setup() {
    Serial.begin(115200);				// Library will begin SoftwareSerial, so we just need to begin Serial

    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line, also begins SoftwareSerial

    // Initializes module using linear phase balancer, and open-drain style driver mode
    pwmController.init(PCA9685_PhaseBalancer_Linear,
                       PCA9685_OutputDriverMode_OpenDrain);

    pwmController.setChannelPWM(0, 2048); // Should see a 50% duty cycle along the 5ms phase width
}

void loop() {
}
