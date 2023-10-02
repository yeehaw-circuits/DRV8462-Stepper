/*

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

#define B2Pattern "%c%c%c%c%c%c%c%c"
#define B2Bin(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

// Easy PWM control using LEDc for this example
unsigned long PWM_Frequency = 2000;
uint8_t PWM_Duty = 127;

  
// Pin definitions - ESP32 default below, change if using a different board or custom pinout.
// #define SPI_MOSI   23
// #define SPI_MISO   19
// #define SPI_SCLK   18
// #define SPI_CS     5

#define DRV_ENABLE  2
#define DRV_nSLEEP  4
#define DRV_nFAULT  15
#define DRV_STEP    16
#define DRV_DIR     17

// Initialize the DRV8462 instance
DRV8462 motorDriver = DRV8462();
// DRV8462 motorDriver = DRV8462(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS); // Custom pinout supported

void setup() {
  Serial.begin(115200); 

  motorDriver.FullInitialize();
  // motorDriver.FullInitialize(DRV_ENABLE, DRV_nSLEEP, DRV_nFAULT, DRV_STEP, DRV_DIR); // Custom pinout supported

  // Optional - Clear any latched faults in the motor driver
  // motorDriver.clearFaultPulse();  // Send a nSLEEP Reset Pulse to clear latched faults per page 67 of datasheet.
  // motorDriver.clearFaultSPI();    // Clear faults via the CLR_FLT bit in CTRL3 register

  // Set the running current
  motorDriver.setRunCurrentPercent(40); // Set run current via TRQ_DAC to 40% = 102/255
  // motorDriver.setTorqueDAC(102);     // Alternatively, set the value of TRQ_DAC directly 0-255
  /*     Recommended starting values below for a few different stepper motors at VM = 12V to 24V 
      Motor size            Percent
      28BYJ-48  (8Ncm  0.2A)  20%?
      NEMA 17   (45Ncm 2.0A)  40%
      NEMA 17   (65Ncm 2.1A)  45% 
      NEMA 23                 50%

  */

  
  motorDriver.setMicrostepMode(MicroStep16); // Set the microstepping mode using names
  // motorDriver.setMicrostepMode(16);       // Set the microstepping mode using a number as argument
  /*  Microstepping Mode Options are below
         Name         #    Description
      MicroStep1_100  1     full-step 100% current
      MicroStep1      71    full-step with 71% current
      MicroStep2      2     1/2 step
      MicroStep2_NC   122   non-circular 1/2 step
      MicroStep4      4     1/4 step 
      MicroStep8      8     1/8 step
      MicroStep16     16    1/16 step
      MicroStep32     32    1/32 step
      MicroStep64     64    1/64 step
      MicroStep128    128   1/128 step
      MicroStep256    256   1/256 step
  */

  // Auto-microstepping: When enabled this feature automatically interpolates the step input pulses to the
  ///   selected auto microstepping modes in the drop down menu of RES_AUTO.
  // Control the driver with the same step pulses for your chosen microstep level (ex. 1/4), 
  ///   but the motor should run smoother and more quietly.  
  motorDriver.enableAutoMicrostepping();      // Enable Auto Microstepping
  // motorDriver.setAutoMicroResolution(256);    // Set the resolution to 1/256 (default), 1/128, 1/64, or 1/32 when auto microstepping enabled. 

  // Standstill Power Saving Feature 
  motorDriver.enableStandstillPowerSaving();  // Enable to use a lower holding current when motor is not moving
  motorDriver.setHoldingCurrentPercent(15);   // Set the Holding Current used if Standstill Power Saving is enabled
  motorDriver.setStandstillDelayTimeMS(32); // Controls the delay between last STEP pulse and activation of standstill power saving mode.  16ms to 1008ms, 64ms is default
  motorDriver.setStandstillDelayTime(4); // Control the delay time via TSTSL_DLY bit directly. 1-63, default is 4 = 64ms (4  *16)
  
  // ################ Optional Motor Driver Settings 
  ///  I recommend leaving these default, only change if you're having issues or want to use that feature.

  // OCP_MODE - Overcurrent Protection Mode
  // motorDriver.setOCP_MODE_autoRecovery();           // Overcurrent condition fault recovery is auto-retry
  // motorDriver.setOCP_MODE_latchedFault();           // (default) - Overcurrent condition causes a latched fault
  
  // Decay Mode - See https://www.ti.com/lit/an/slvae80a/slvae80a.pdf
  // motorDriver.setDecayMode(SmartTuneDynamicDecay); // Set the Decay Mode. 
  //   ^Options are SlowDecay, Mixed30Decay, Mixed60Decay, SmartTuneDynamicDecay, SmartTuneRippleControl (default)

  // TBLANK - Set the current sense blanking time via TBLANK_TIME bit. 
  // motorDriver.setTBLANKTime_us(20); // Input 10 = 1.0us, 15 = 1.5us(default), 20 = 2.0us, or 25 = 2.5us
  // ################ End Optional Motor Driver Settings ################

  // Slew Rate
  // motorDriver.setOutputSlewRate(70); // Set the Output rise/fall time to 140ns (default) or 70ns

  // T_OFF time - Set the PWM OFF time in microseconds (us)
  // Note - T_OFF time is not used for smart tune ripple control and silent step decay modes.
  // motorDriver.setTOFF(27); // Options are 9 us, 19 us (default), 27 us, or 35 us. Or 0/1/2/3 respectively.
  
  // Sleep mode
  // motorDriver.wakeup_nSLEEP_HIGH(); // Sets the nSLEEP pin HIGH to wake up the DRV8462
  // motorDriver.asleep_nSLEEP_LOW();  // Sets the nSLEEP pin LOW to enter low-power sleep mode

  // Auto Torque - See AutoTorque.ino example 
  
  // Enable the motor driver outputs, starts running current through the motor but doesn't move until STEP pulse. 
  motorDriver.enableOutputs(); // Set the ENABLE pin HIGH

  // Set the direction
  motorDriver.setDirectionForward();
  // motorDriver.setDirectionReverse();
  // motorDriver.toggleDirection();

  // Easy PWM generation using LEDc on ESP32
  ledcSetup(0, PWM_Frequency, 8);  // Channel 0, default to 5000kHz, 8-bit resolution (0-255)
  ledcAttachPin(DRV_STEP, 0);
  // For Non-ESP32 boards:  pinMode(DRV_STEP, OUTPUT);

}

void loop() {
  // Enable STEP signal at 50% duty cycle for 1 second
  ledcWrite(0, 127);  // For Non-ESP32:  analogWrite(DRV_STEP, 127);
  delay(2000);

  // Step sending the STEP signal (stop the motor)
  ledcWrite(0, 0); // For Non-ESP32:  analogWrite(DRV_STEP, 0);
  delay(500);

  motorDriver.toggleDirection(); // Change Direction

  // Enable STEP signal at 50% duty cycle for 1 second
  ledcWrite(0, 127);
  delay(2000);

  // Stop the STEP signal
  ledcWrite(0, 0);

  // Read values of registers
  Serial.print("Fault Register: "); Serial.println(motorDriver.readRegister(0));
  Serial.print("Diag2 Register: "); Serial.println(motorDriver.readRegister(2));

  // Wait 1 second
  delay(1000);
  motorDriver.toggleDirection(); // Change Direction

}


