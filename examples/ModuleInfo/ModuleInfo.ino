// PCA9685-Arduino Module Info
// In this example, we enable debug output support. We copy the PCA9685_ENABLE_DEBUG_OUTPUT
// define from the libraries header file for debug output support to enable and the
// printModuleInfo() method to become available. Calling this method will display
// information about the module itself, including initalized states, register values,
// current settings, etc. All library calls being made will also display internal debug
// information about the structure of the call itself.

// Uncomment this define to enable debug output.
#define PCA9685_ENABLE_DEBUG_OUTPUT     1

#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwmController;

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmController.printModuleInfo();
}

void loop() {
}
