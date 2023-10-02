/*
  This file shows the bare minimum needed to run a stepper motor with this library.
  See the example "DRV8462_GettingStarted.ino" for more details on the functions and 
  capabilities of this motor driver and library.  

My wiring for 30-pin version of ESP32 - https://www.amazon.com/dp/B08246MCL5
//
//                         /-uUSB-\
//                   /-[O]-----------[O]-\
//       +3.3V RAIL -|3.3V        (5V)VIN|-
//         GND RAIL -|GND             GND|-
//           nFAULT -|GPIO15       GPIO13|-
//           ENABLE -|GPIO02       GPIO12|-
//           nSLEEP -|GPIO04       GPIO14|-
//             STEP -|GPIO16       GPIO27|-
//              DIR -|GPIO17       GPIO26|- 
//               CS -|GPIO05       GPIO25|- 
//              CLK -|GPIO18       GPIO33|-                              (don't use with WiFi)
// DRV8462 SDO/MISO -|GPIO19       GPIO32|-                              (don't use with WiFi)
//                  -|GPIO21(SDA)  GPIO35|-                  (input only)(don't use with WiFi)
//                  -|GPIO03(RX)   GPIO34|-                  (input only)(don't use with WiFi)
//                  -|GPIO01(TX)   GPIO39|-                  (input only)(don't use with WiFi)
//                  -|GPIO22(SCL)  GPIO36|-                  (input only)(don't use with WiFi)
// DRV8462 SDI/MOSI -|GPIO23           EN|-
//                   \-------------------/
//
// ESP32 Pinout Reference - which GPIOs to use and not use - https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// NOTE - ADC2 pins cannot be used for Analog Input when Wi-Fi is used!! And GPIO34-39 are inputs only, can't be outputs ever.  
*/

#include "Arduino.h"            // Arduino Core
#include <DRV8462.h>            // Custom library for DRV8462

// ESP32 PWM output for STEP signal using LEDc
unsigned long PWM_Frequency = 1500;
#define DRV_STEP    16

// Initialize the DRV8462 instance
DRV8462 motorDriver = DRV8462();

void setup() {
  Serial.begin(115200); 
  motorDriver.FullInitialize();

  // Set the running current
  motorDriver.setRunCurrentPercent(20);

  // Set the Microstepping level (DRV8462 defaults to 1/16 step)
  motorDriver.setMicrostepMode(MicroStep32); // See DRV8462_GettingStarted.ino example for microstepping options

  motorDriver.enableOutputs(); // Set the ENABLE pin HIGH

  // #################### Optional Motor Driver Settings ############
  // motorDriver.setDirectionForward();
  // motorDriver.setDirectionReverse();
  // motorDriver.enableAutoMicrostepping();             // Enable Automatic Microstepping where the driver interpolates between STEP signals
  // motorDriver.setAutoMicroResolution(256);           // Set the resolution to 1/256 (default), 1/128, 1/64, or 1/32 when auto microstepping enabled. 
  // motorDriver.setOCP_MODE_autoRecovery();            // DRV8462 default OCP mode is Latched Fault
  // motorDriver.setDecayMode(SmartTuneRippleControl);  // Options are SlowDecay, Mixed30Decay, Mixed60Decay, SmartTuneDynamicDecay, SmartTuneRippleControl (default)
  // motorDriver.enableStandstillPowerSaving();         // Enable to use a lower holding current when motor is not moving
  // motorDriver.setHoldingCurrentPercent(5);           // Set the Holding Current used if Standstill Power Saving is enabled
  
  // ESP32 - Easy PWM generation using LEDc - https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  ledcSetup(0, PWM_Frequency, 8);  // Channel 0, default to 1000kHz, 8-bit resolution (0-255)
  ledcAttachPin(DRV_STEP, 0);
  ledcWrite(0, 127); // 50% duty cycle

  // Other Arduino boards - use AnalogWrite at fixed frequency instead - https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
  // analogWrite(DRV_STEP, 127);

}

void loop() {
  motorDriver.toggleDirection();
  delay(1000);

  motorDriver.toggleDirection();
  delay(1000);
}


