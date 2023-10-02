/*!
 * @file DRV8462.cpp
 *
 * @mainpage Adafruit BMP183 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Adafruit BMP183 Barometric Pressure + Temp sensor
 *
 * Designed specifically to work with the Adafruit BMP183 Breakout
 * ----> http://www.adafruit.com/products/1900
 *
 * These sensors use SPI to communicate, 4 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "DRV8462.h"
#ifdef __AVR__
#include <util/delay.h>
#endif
#define _delay_ms(t)                                                           \
  delay(t) //!< Delay for use in-between different lines/parts of code
#include <SPI.h>

volatile DRV8xx2_REG_t gDeviceSpiReg;

/*!
 *  @brief  Sets the hardware pins used by the driver
 *  @param  enablePin
 *          DRV8462 ENABLE pin on your microcontroller
 *  @param  nSleepPin
 *          DRV8462 nSLEEP pin on your microcontroller
 *  @param nFaultPin
 *          DRV8462 nFAULT pin on your microcontroller
 *  @param stepPin
 *          DRV8462 STEP pin
 *  @param dirPin
 *          DRV8462 DIR pin
 */
void DRV8462::setPins( int8_t enablePin, int8_t nSleepPin, int8_t nFaultPin, 
              int8_t stepPin, int8_t dirPin)
{
  _enable = enablePin;
  _nsleep = nSleepPin;
  _nfault = nFaultPin;
  _step   = stepPin;
  _dir    = dirPin;

  // Pin Modes
  pinMode(_nfault, INPUT);   // Fault indication output. Pulled logic low with fault condition. 
  pinMode(_enable, OUTPUT);  digitalWrite(_enable,  0);  // Logic low to disable device outputs; logic high to enable.
  pinMode(_nsleep, OUTPUT);  digitalWrite(_nsleep,  0);  // Logic high to enable device; logic low to enter low-power sleep mode. A narrow nSLEEP reset pulse clears latched faults.
  pinMode(_step,   OUTPUT);  digitalWrite(_step,    0); 
  pinMode(_dir,    OUTPUT);  digitalWrite(_dir,     0);

}

// Initialize SPI, inputs, outputs, and wake up the device. 
void DRV8462::FullInitialize(int8_t enablePin, int8_t nSleepPin, int8_t nFaultPin, int8_t stepPin, int8_t dirPin)
{
  setPins(enablePin, nSleepPin, nFaultPin, stepPin, dirPin);
  beginSPI();
  wakeup_nSLEEP_HIGH();
  setMicrostepMode(MicroStep16);
  clearFaults();
}

/*!
 *  @brief  Sets the ENABLE pin HIGH to enable the DRV8462 motor outputs
 */
void DRV8462::enableOutputs()
{
  // Logic low to disable device outputs; logic high to enable.
  pinMode(_enable, OUTPUT);  
  digitalWrite(_enable,  1);
}

/*!
 *  @brief  Sets the ENABLE pin LOW to disable the DRV8462 motor outputs
 */
void DRV8462::disableOutputs()
{
  // Logic low to disable device outputs; logic high to enable.
  pinMode(_enable, OUTPUT);  
  digitalWrite(_enable,  0);
}

/*!
 *  @brief  Set the DIR pin HIGH
 */
void DRV8462::setDirectionForward()
{
  // Logic high for forward, following the sequence table in the datasheet
  pinMode(_dir, OUTPUT);  
  digitalWrite(_dir,  1);
}

/*!
 *  @brief  Set the DIR pin LOW
 */
void DRV8462::setDirectionReverse()
{
  // Logic low for reverse
  pinMode(_dir, OUTPUT);  
  digitalWrite(_dir,  0);
}

/*!
 *  @brief  Toggle the DIR pin
 */
void DRV8462::toggleDirection()
{
  pinMode(_dir, OUTPUT);  
  digitalWrite(_dir,  !digitalRead(_dir));
}


/*!
 *  @brief  Sets the nSLEEP pin HIGH to wake up the DRV8462
 */
void DRV8462::wakeup_nSLEEP_HIGH()
{
  // Logic high to enable device; logic low to enter low-power sleep mode. 
  // A narrow nSLEEP reset pulse clears latched faults.
  pinMode(_nsleep, OUTPUT);  
  digitalWrite(_nsleep,  1);

  // Set EN_OUT bit to 1
  uint8_t EN_OUT_only = (gDeviceSpiReg.ctrl1_reg |= (1 << 7)); // bitshift by 7 to get just EN_OUT (bit 7)
  
  writeRegister(SPI_CTRL1, EN_OUT_only);
  gDeviceSpiReg.ctrl1_reg = EN_OUT_only; // Update ESP32 copy of DRV8462 registers
}

/*!
 *  @brief  Sets the nSLEEP pin LOW to enter low-power sleep mode
 */
void DRV8462::asleep_nSLEEP_LOW()
{
  // Logic high to enable device; logic low to enter low-power sleep mode. 
  // A narrow nSLEEP reset pulse clears latched faults.
  pinMode(_nsleep, OUTPUT);  
  digitalWrite(_nsleep,  0);
}

/*!
 *  @brief  Send a nSLEEP Reset Pulse per page 67 of datasheet
 */
void DRV8462::clearFaultPulse()
{
  pinMode(_nsleep, OUTPUT);  
  digitalWrite(_nsleep,  0);
  delayMicroseconds(25);
  digitalWrite(_nsleep,  1);  // Logic high to enable device
}

/*!
 *  @brief  Clear faults via CTRL3 register
 */
void DRV8462::clearFaultSPI()
{
  writeRegister(SPI_CTRL3, 0xB8); // Clear fault via CTRL3 
  gDeviceSpiReg.ctrl3_reg = 0x38; // CLR_FLT bit automatically resets to 0
}

/*!
 *  @brief  Clear faults via CTRL3 register
 */
void DRV8462::clearFaults()
{
  clearFaultSPI();
}

/*!
 *  @brief  Set the Torque DAC value from 0 to 255
 */
void DRV8462::setTorqueDAC(uint8_t newDACValue0to255)
{
  newDACValue0to255 = constrain(newDACValue0to255, 0, 255);
  
  writeRegister(SPI_CTRL11, newDACValue0to255);
  gDeviceSpiReg.ctrl11_reg = newDACValue0to255;
}

/*!
 *  @brief  Set the Torque DAC value from 0 to 100 percent
 */
void DRV8462::setTorqueDACPercent(uint8_t newDACValue0to100)
{
  newDACValue0to100 = constrain(newDACValue0to100, 0, 100);
  uint8_t newDACValue0to255 = map(newDACValue0to100, 0, 100, 0, 255);

  writeRegister(SPI_CTRL11, newDACValue0to255);
  gDeviceSpiReg.ctrl11_reg = newDACValue0to255;
}

/*!
 *  @brief  Set the Torque DAC value from 0 to 100 percent
 */
void DRV8462::setRunCurrentPercent(uint8_t newDACValue0to100)
{
  setTorqueDACPercent(newDACValue0to100);
}

/*!
 *  @brief  Enable Automatic Microstepping by setting EN_AUTO bit in CTRL9
 */
void DRV8462::enableAutoMicrostepping()
{
  uint8_t EN_AUTO_only = (gDeviceSpiReg.ctrl9_reg |= EN_AUTO_MASK);
  
  writeRegister(SPI_CTRL9, EN_AUTO_only);
  gDeviceSpiReg.ctrl9_reg = EN_AUTO_only;
}

/*!
 *  @brief  Disable Automatic Microstepping by clearing EN_AUTO bit in CTRL9
 */
void DRV8462::disableAutoMicrostepping()
{
  uint8_t EN_AUTO_only = (gDeviceSpiReg.ctrl9_reg &= ~EN_AUTO_MASK);

  writeRegister(SPI_CTRL9, EN_AUTO_only);
  gDeviceSpiReg.ctrl9_reg = EN_AUTO_only;
}

// Set the resolution of Auto Microstepping with a number input
void DRV8462::setAutoMicroResolution(uint16_t modeNum)
{
  uint8_t registerValue = 0;
  switch(modeNum)
  {
    case 0:
    case 256:
      // 1/256 auto microstepping
      registerValue = 0;
      break;
    
    case 1:
    case 128:
      // 1/128 auto microstepping
      registerValue = 1;
      break;
    
    case 2:
    case 64:
      // 1/64
      registerValue = 2;
      break;
    
    case 3:
    case 32:
      // 1/32
      registerValue = 3;
      break;
    
    default:
      // Default to 1/256 microstepping
      registerValue = 0;
      break;
  }

  // Send the SPI command
  uint8_t RES_AUTO_only = (gDeviceSpiReg.ctrl9_reg & ~RES_AUTO_MASK | (registerValue << 1));  // bitshift by 1 to get just RES_AUTO bits 2:1

  writeRegister(SPI_CTRL9, RES_AUTO_only);
  gDeviceSpiReg.ctrl9_reg = RES_AUTO_only; // Update ESP32 copy of DRV8462 registers

}

// Set the resolution of Auto Microstepping with a number input
// The DRV8462 defaults to 140ns
void DRV8462::setOutputSlewRate(uint8_t modeNum)
{
  uint8_t registerValue = 0;
  switch(modeNum)
  {
    case 0:
    case 140:
      // Output rise/fall time 140ns
      registerValue = 0;
      break;
    
    case 1:
    case 70:
      // Output rise/fall time 70ns
      registerValue = 1;
      break;
    
    default:
      // Default to 140ns
      registerValue = 0;
      break;
  }

  // Send the SPI command
  uint8_t SR_only = (gDeviceSpiReg.ctrl1_reg & ~SR_MASK | (registerValue << 6)); // bitshift by 6 to get just SR (bit 6)
  writeRegister(SPI_CTRL1, SR_only);
  gDeviceSpiReg.ctrl1_reg = SR_only; // Update ESP32 copy of DRV8462 registers

}

// Set the T_OFF time
// The DRV8462 defaults to 19us
void DRV8462::setTOFF(uint8_t modeNum)
{
  uint8_t registerValue = 0;
  switch(modeNum)
  {
    case 0:
    case 9:
      // TOFF of 9us
      registerValue = 0;
      break;
    
    case 1:
    case 19:
      // TOFF of 19us
      registerValue = 1;
      break;

    case 2:
    case 27:
      // TOFF of 27us
      registerValue = 2;
      break;
    case 3:
    case 35:
      // TOFF of 35us
      registerValue = 3;
      break;
    default:
      // Default to 140ns
      registerValue = 0;
      break;
  }

  // Send the SPI command
  uint8_t TOFF_only = (gDeviceSpiReg.ctrl1_reg & ~TOFF_MASK | (registerValue << 3)); // Set only just TOFF [bits 4:3]
  writeRegister(SPI_CTRL1, TOFF_only);
  gDeviceSpiReg.ctrl1_reg = TOFF_only; // Update ESP32 copy of DRV8462 registers

}

/*!
 *  @brief  Set Overcurrent condition to cause automatic recovery by setting OCP_MODE bit in CTRL3
 */
void DRV8462::setOCP_MODE_autoRecovery()
{
  uint8_t OCP_MODE_only = (gDeviceSpiReg.ctrl3_reg |= OCP_MODE_MASK);
  
  writeRegister(SPI_CTRL3, OCP_MODE_only);
  gDeviceSpiReg.ctrl3_reg = OCP_MODE_only;
}

/*!
 *  @brief  Set Overcurrent condition to cause automatic recovery by setting OCP_MODE bit in CTRL3 (DRV8462 Default)
 */
void DRV8462::setOCP_MODE_latchedFault()
{
  uint8_t OCP_MODE_only = (gDeviceSpiReg.ctrl3_reg &= ~OCP_MODE_MASK);
  
  writeRegister(SPI_CTRL3, OCP_MODE_only);
  gDeviceSpiReg.ctrl3_reg = OCP_MODE_only;
}

/*!
 *  @brief  Enable Standstill Power Saving by setting the EN_STSL bit in CTRL12
 */
void DRV8462::enableStandstillPowerSaving()
{
  uint8_t EN_STSL_only = (gDeviceSpiReg.ctrl12_reg |= EN_STSL_MASK);
  
  writeRegister(SPI_CTRL12, EN_STSL_only);
  gDeviceSpiReg.ctrl12_reg = EN_STSL_only;
}

/*!
 *  @brief  Enable Standstill Power Saving by clearing the EN_STSL bit in CTRL12
 */
void DRV8462::disableStandstillPowerSaving()
{
  uint8_t EN_STSL_only = (gDeviceSpiReg.ctrl12_reg &= ~EN_STSL_MASK);

  writeRegister(SPI_CTRL12, EN_STSL_only);
  gDeviceSpiReg.ctrl12_reg = EN_STSL_only;
}

void DRV8462::setHoldingCurrent(uint8_t newHoldingCurrent)
{
  writeRegister(SPI_CTRL10, newHoldingCurrent);
  gDeviceSpiReg.ctrl10_reg = newHoldingCurrent;
}

void DRV8462::setHoldingCurrentPercent(uint8_t newHoldingCurrentPercent)
{
  newHoldingCurrentPercent = constrain(newHoldingCurrentPercent, 0, 100);
  uint8_t newHoldCurrent0to255 = map(newHoldingCurrentPercent, 0, 100, 0, 255);

  writeRegister(SPI_CTRL10, newHoldCurrent0to255);
  gDeviceSpiReg.ctrl10_reg = newHoldCurrent0to255;
}


/*!
 *  @brief  Controls the delay between last STEP pulse and activation of standstill power saving mode
 *  @param  newDelayTime
 *          Value from 0 to 63 for setting the bits directly
 */
void DRV8462::setStandstillDelayTime(uint16_t newDelayTime)
{
  // DRV8462 default is 4, which is 64ms
  newDelayTime = constrain(newDelayTime, 1, 63);

  uint8_t TSTSL_DLY_only = (gDeviceSpiReg.ctrl13_reg & ~TSTSL_DLY_MASK | (newDelayTime << 2)); // Set only just TSTSL_DLY [bits 7:2]
  writeRegister(SPI_CTRL13, TSTSL_DLY_only);
  gDeviceSpiReg.ctrl13_reg = TSTSL_DLY_only;
}

/*!
 *  @brief  Controls the delay between last STEP pulse and activation of standstill power saving mode
 *  @param  newDelayTime
 *          Value from 16ms to 1008ms for the delay time. Default is 64ms
 */
void DRV8462::setStandstillDelayTimeMS(uint16_t newDelayTimeMilliseconds)
{
  newDelayTimeMilliseconds = constrain(newDelayTimeMilliseconds, 16, 1008);

  uint8_t newDelayTime = (int)(newDelayTimeMilliseconds/16);

  uint8_t TSTSL_DLY_only = (gDeviceSpiReg.ctrl13_reg & ~TSTSL_DLY_MASK | (newDelayTime << 2)); // Set only just TSTSL_DLY [bits 7:2]
  writeRegister(SPI_CTRL13, TSTSL_DLY_only);
  gDeviceSpiReg.ctrl13_reg = TSTSL_DLY_only;
}

/*!
 *  @brief  Controls the current sense blanking time
 *  @param  newDelayTime_us
 *          Input 10 (for 1.0us), 15 (for 1.5us), 2 (2.0 us), or 25 (for 2.5us)
 */
void DRV8462::setTBLANKTime_us(uint8_t newDelayTime_us)
{
  uint8_t registerValue = 0;
  switch(newDelayTime_us)
  {
    case 0:
    case 1:
    case 10:
      // 1.0 us blanking time
      registerValue = 0;
      break;
    case 15:
      // 1.5 us blanking time
      registerValue = 1;
      break;
    case 2:
    case 20:
      // 2.0 us blanking time
      registerValue = 2;
      break;
    case 3:
    case 25:
      // 2.5 us blanking time
      registerValue = 3;
      break;
    default:
      registerValue = 0;
      break;
  }

  uint8_t TBLANK_only = (gDeviceSpiReg.ctrl4_reg & ~TBLANK_TIME_MASK | (registerValue << 6)); 
  writeRegister(SPI_CTRL4, TBLANK_only);
  gDeviceSpiReg.ctrl4_reg = TBLANK_only;
}





/*!
 *  @brief  Instantiates a new DRV8462 class using software SPI
 *  @param  SPICLK
 *          SPI chip clock
 *  @param  SPIMISO
 *          SPI MISO (Data to microcontroller from sensor)
 *  @param  SPIMOSI
 *          SPI MOSI (Data from microcontroller to sensor)
 *  @param  SPICS
 *          SPI CS PIN
 */
DRV8462::DRV8462(int8_t SPICLK, int8_t SPIMISO, int8_t SPIMOSI, int8_t SPICS) {
  _cs = SPICS;
  _clk = SPICLK;
  _miso = SPIMISO;
  _mosi = SPIMOSI;
}


/*!
 *  @brief  Setups the HW
 *  @param  mode
 *          selected BMP183 mode
 *  @return true if successful
 */
boolean DRV8462::beginSPI() {
  // Set nSLEEP to HIGH to wake up the driver
  pinMode(_nsleep, OUTPUT);  
  digitalWrite(_nsleep,  1);

  if (_clk == -1) {
    _spi->begin();
    _spi->setDataMode(SPI_MODE1);
#ifdef __AVR__
    _spi->setClockDivider(SPI_CLOCK_DIV16);
#endif
#ifdef __SAM3X8E__
    _spi->setClockDivider(11); // 8-ish MHz (full! speed!)
#endif
  } 
  else {
    pinMode(_clk, OUTPUT);
    digitalWrite(_clk, LOW);
    pinMode(_mosi, OUTPUT);
    digitalWrite(_mosi, HIGH);
    pinMode(_miso, INPUT);
  }
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  delay(1);

  // Set EN_OUT bit to 1
  uint8_t EN_OUT_only = (gDeviceSpiReg.ctrl1_reg |= (1 << 7)); // bitshift by 7 to get just EN_OUT (bit 7)
  writeRegister(SPI_CTRL1, EN_OUT_only);
  gDeviceSpiReg.ctrl1_reg = EN_OUT_only; // Update ESP32 copy of DRV8462 registers
  return true;
}


void DRV8462::setMicrostepMode(DRV8462_microstep_t microstep)
{
  /* Microstep boundary check */
  if ((microstep > MicroStep256) || (microstep < 0)) {
    microstep = MicroStep256;
  }

  // Send the SPI command
  uint8_t MICROSTEP_only = (gDeviceSpiReg.ctrl2_reg & ~MICROSTEP_MODE_MASK) | microstep;
  writeRegister(SPI_CTRL2, MICROSTEP_only);
  gDeviceSpiReg.ctrl2_reg = MICROSTEP_only; // Update ESP32 copy of DRV8462 registers
}

// Set the microstepping mode with a normal number input
void DRV8462::setMicrostepMode(uint16_t modeNum)
{
  uint8_t registerValue = 0;
  switch(modeNum)
  {
    case 1:
    case 100:
      // Full step with 100% current
      registerValue = MicroStep1_100;
      break;
    
    case 71:
      /// Full step with 71% current
      registerValue = MicroStep1;
      break;
    
    case 2:
    case 12:
      // Circular 1/2 step
      registerValue = MicroStep2;
      break;
    
    case 122:
      // Non-circular 1/2 step
      registerValue = MicroStep2_NC;
      break;
    
    case 4:
    case 14:
      // 1/4 step
      registerValue = MicroStep4;
      break;
    
    case 8:
    case 18:
      registerValue = MicroStep8;
      break;
    
    case 16:
    case 116:
      registerValue = MicroStep16;
      break;
    case 32:
    case 132:
      registerValue = MicroStep32;
      break;
    case 64:
    case 164:
      registerValue = MicroStep64;
      break;
    case 128:
    case 1128:
      registerValue = MicroStep128;
      break;
    case 256:
    case 1256:
      registerValue = MicroStep256;
      break;

    default:
      registerValue = MicroStep16;
      break;
  }

  // Send the SPI command
  uint8_t MICROSTEP_only = (gDeviceSpiReg.ctrl2_reg & ~MICROSTEP_MODE_MASK) | registerValue;
  writeRegister(SPI_CTRL2, MICROSTEP_only);
  gDeviceSpiReg.ctrl2_reg = MICROSTEP_only; // Update ESP32 copy of DRV8462 registers

}

void DRV8462::setDecayMode(DRV8462_decay_t decayMode)
{
  /* Microstep boundary check */
  if ((decayMode > SmartTuneRippleControl) || (decayMode < 0)) {
    decayMode = SmartTuneRippleControl;
  }

  // Send the SPI command
  uint8_t DECAY_only = (gDeviceSpiReg.ctrl1_reg & ~DECAY_MASK) | decayMode;
  writeRegister(SPI_CTRL1, DECAY_only);
  gDeviceSpiReg.ctrl1_reg = DECAY_only; // Update ESP32 copy of DRV8462 registers
}


// ################################## AUTO TORQUE ###########################

/*!
 *  @brief  Enable Auto Torque by setting the ATQ_EN bit
 */
void DRV8462::enableAutoTorqueMode()
{
  // uint8_t ATQ_only = (gDeviceSpiReg.atq_ctrl10_reg |= ATQ_EN_MASK);
  uint8_t registerNow = readRegister(SPI_ATQ_CTRL10);
  uint8_t ATQ_only = (registerNow |= ATQ_EN_MASK);
  
  writeRegister(SPI_ATQ_CTRL10, ATQ_only);
  gDeviceSpiReg.atq_ctrl10_reg = ATQ_only;
}

/*!
 *  @brief  Disable Auto Torque by clearing the ATQ_EN bit
 */
void DRV8462::disableAutoTorqueMode()
{
  // uint8_t ATQ_only = (gDeviceSpiReg.atq_ctrl10_reg &= ~ATQ_EN_MASK);
  uint8_t registerNow = readRegister(SPI_ATQ_CTRL10);
  uint8_t ATQ_only = (registerNow &= ~ATQ_EN_MASK);
  
  writeRegister(SPI_ATQ_CTRL10, ATQ_only);
  gDeviceSpiReg.atq_ctrl10_reg = ATQ_only;
}

/*!
 *  @brief  Begin the Auto Torque learning process by setting LRN_START bit
 */
void DRV8462::ATQ_StartLearning()
{
  // uint8_t ATQ_only = (gDeviceSpiReg.atq_ctrl10_reg |= LRN_START_MASK);
  uint8_t registerNow = readRegister(SPI_ATQ_CTRL10);
  uint8_t ATQ_only = (registerNow |= LRN_START_MASK);
  
  writeRegister(SPI_ATQ_CTRL10, ATQ_only);
  // gDeviceSpiReg.atq_ctrl10_reg = ATQ_only; // bit automatically resets to 0
}

/*!
 *  @brief  Read the LRN_START bit to see when learning is complete
 */
uint8_t DRV8462::ATQ_ReadLRN_START()
{
  uint8_t value = (readRegister(SPI_ATQ_CTRL10) & LRN_START_MASK) >> 6;
  // gDeviceSpiReg.atq_ctrl10_reg = value; // figure out later
  return value;
}

/*!
 *  @brief  Read the LRN_DONE bit to see when learning is complete
 */
uint8_t DRV8462::ATQ_ReadLRN_DONE()
{
  uint8_t value = (readRegister(SPI_DIAG2) & ATQ_LRN_DONE_MASK) >> 2;
  // gDeviceSpiReg.atasdfasdf0_reg = value; // figure out later
  return value;
}


/*!
 *  @brief  Enable Auto Torque VM Scaling by setting the ATQ_VM_SCALE bit
 */
void DRV8462::ATQ_enableVMScaling()
{
  // uint8_t ATQ_only = (gDeviceSpiReg.atq_ctrl17_reg |= ATQ_VM_SCALE_MASK);
  uint8_t registerNow = readRegister(SPI_ATQ_CTRL17);
  uint8_t ATQ_only = (registerNow |= ATQ_VM_SCALE_MASK);
  
  writeRegister(SPI_ATQ_CTRL17, ATQ_only);
  gDeviceSpiReg.atq_ctrl17_reg = ATQ_only;
}

/*!
 *  @brief  Disable Auto Torque by clearing the ATQ_VM_SCALE bit
 */
void DRV8462::ATQ_disableVMScaling()
{
  uint8_t registerNow = readRegister(SPI_ATQ_CTRL17);
  uint8_t ATQ_only = (registerNow &= ~ATQ_VM_SCALE_MASK);
  
  writeRegister(SPI_ATQ_CTRL17, ATQ_only);
  gDeviceSpiReg.atq_ctrl17_reg = ATQ_only;
}

/*!
 *  @brief  Set the intial learn current via ATQ_LRN_MIN_CURRENT
 */
void DRV8462::ATQ_SetInitialLearnCurrent(uint8_t newValue)
{
  newValue = constrain(newValue, 0, (0b11111));

  uint8_t registerNow = readRegister(SPI_ATQ_CTRL4);
  uint8_t ATQ_only = ((registerNow & ~ATQ_LRN_MIN_CURRENT_MASK )| (newValue << 3)); // Set only just ATQ_LRN_MIN_CURRENT [bits 7:3]
  writeRegister(SPI_ATQ_CTRL4, ATQ_only);
  gDeviceSpiReg.atq_ctrl4_reg = ATQ_only;
}

/*!
 *  @brief  Set the increment to initial current level via ATQ_LRN_STEP
 */
void DRV8462::ATQ_SetCurrentStepsForLearn(uint8_t newValue)
{

  uint8_t registerValue = 0;
  switch(newValue)
  {
    case 0:
    case 128:
      // ATQ_LRN_STEP = 128
      registerValue = 0;
      break;
    case 1:
    case 16:
      // ATQ_LRN_STEP = 16
      registerValue = 1;
      break;
    case 2:
    case 32:
      // ATQ_LRN_STEP = 32
      registerValue = 2;
      break;
    case 3:
    case 64:
      // ATQ_LRN_STEP = 64
      registerValue = 3;
      break;
    default:
      registerValue = 0;
      break;
  }

  // uint8_t ATQ_only = (gDeviceSpiReg.atq_ctrl15_reg & ~ATQ_LRN_STEP_MASK | (registerValue << 2)); // Set only just ATQ_LRN_STEP [bits 3:2]
  uint8_t registerNow = readRegister(SPI_ATQ_CTRL15);
  uint8_t ATQ_only = (registerNow & ~ATQ_LRN_STEP_MASK | (registerValue << 2)); // Set only just ATQ_LRN_STEP [bits 3:2]
  writeRegister(SPI_ATQ_CTRL15, ATQ_only);
  gDeviceSpiReg.atq_ctrl15_reg = ATQ_only;
}

/*!
 *  @brief  Set the increment to initial current level via ATQ_LRN_STEP
 */
void DRV8462::ATQ_SetCurrentCyclesForLearn(uint8_t newValue)
{

  uint8_t registerValue = 0;
  switch(newValue)
  {
    case 0:
    case 8:
      // ATQ_LRN_CYCLE_SELECT = 0b00 : 8 half-cycles
      registerValue = 0;
      break;
    case 1:
    case 16:
      // ATQ_LRN_CYCLE_SELECT = 0b01 : 16 half-cycles
      registerValue = 1;
      break;
    case 2:
    case 24:
      // ATQ_LRN_CYCLE_SELECT = 0b10 : 24 half-cycles
      registerValue = 2;
      break;
    case 3:
    case 32:
      // ATQ_LRN_CYCLE_SELECT = 0b11 : 32 half-cycles
      registerValue = 3;
      break;
    default:
      registerValue = 0;
      break;
  }
  // uint8_t ATQ_only = (gDeviceSpiReg.atq_ctrl15_reg & ~ATQ_LRN_CYCLE_SELECT_MASK | (registerValue )); // Set only just ATQ_LRN_CYCLE_SELECT [bits 1:0]
  uint8_t registerNow = readRegister(SPI_ATQ_CTRL15);
  uint8_t ATQ_only = (registerNow & ~ATQ_LRN_CYCLE_SELECT_MASK | (registerValue )); // Set only just ATQ_LRN_CYCLE_SELECT [bits 1:0]
  writeRegister(SPI_ATQ_CTRL15, ATQ_only);
  gDeviceSpiReg.atq_ctrl15_reg = ATQ_only;
}

/*!
 *  @brief  Set the maximum coil current via ATQ_TRQ_MAX. Defaults to 255/255
 */
void DRV8462::ATQ_SetMaxCurrentLimit(uint8_t newValue)
{
  writeRegister(SPI_ATQ_CTRL12, newValue);
  gDeviceSpiReg.atq_ctrl12_reg = newValue;
}

/*!
 *  @brief  Set the maximum coil current % via ATQ_TRQ_MAX. Defaults to 100%
 */
void DRV8462::ATQ_SetMaxCurrentLimitAsPercent(uint8_t newValue0to100)
{
  newValue0to100 = constrain(newValue0to100, 0, 100);
  uint8_t newValue0to255 = map(newValue0to100, 0, 100, 0, 255);
  ATQ_SetMaxCurrentLimit(newValue0to255);
}

/*!
 *  @brief  Read ATQ_TRQ_MAX
 */
uint8_t DRV8462::ATQ_ReadMaxCurrentLimit()
{
  uint8_t value = readRegister(SPI_ATQ_CTRL12);
  gDeviceSpiReg.atq_ctrl12_reg = value;
  return value;
}

/*!
 *  @brief  Set the minimum coil current via ATQ_TRQ_MIN. Defaults to 10/255
 */
void DRV8462::ATQ_SetMinCurrentLimit(uint8_t newValue)
{
  writeRegister(SPI_ATQ_CTRL11, newValue);
  gDeviceSpiReg.atq_ctrl11_reg = newValue;
}

/*!
 *  @brief  Set the minimum coil current % via ATQ_TRQ_MIN. Defaults to 3.9%
 */
void DRV8462::ATQ_SetMinCurrentLimitAsPercent(uint8_t newValue0to100)
{
  newValue0to100 = constrain(newValue0to100, 0, 100);
  uint8_t newValue0to255 = map(newValue0to100, 0, 100, 0, 255);
  ATQ_SetMinCurrentLimit(newValue0to255);
}

/*!
 *  @brief  Read ATQ_TRQ_MIN
 */
uint8_t DRV8462::ATQ_ReadMinCurrentLimit()
{
  uint8_t value = readRegister(SPI_ATQ_CTRL11);
  gDeviceSpiReg.atq_ctrl11_reg = value;
  return value;
}

/*!
 *  @brief  Programs the upper limit of the auto torque hysteretic band via ATQ_UL
 */
void DRV8462::ATQ_SetATQCountUpperLimit(uint8_t newValue)
{
  writeRegister(SPI_ATQ_CTRL6, newValue);
  gDeviceSpiReg.atq_ctrl6_reg = newValue;
}

/*!
 *  @brief  Programs the upper limit as % of the auto torque hysteretic band via ATQ_UL
 */
void DRV8462::ATQ_SetATQCountUpperLimitAsPercent(uint8_t newValue0to100)
{
  newValue0to100 = constrain(newValue0to100, 0, 100);
  uint8_t newValue0to255 = map(newValue0to100, 0, 100, 0, 255);
  ATQ_SetATQCountUpperLimit(newValue0to255);
}

/*!
 *  @brief  Read ATQ_UL
 */
uint8_t DRV8462::ATQ_ReadATQCountUpperLimit()
{
  uint8_t value = readRegister(SPI_ATQ_CTRL6);
  gDeviceSpiReg.atq_ctrl6_reg = value;
  return value;
}

/*!
 *  @brief  Programs the lower limit of the auto torque hysteretic band via ATQ_LL
 */
void DRV8462::ATQ_SetATQCountLowerLimit(uint8_t newValue)
{
  writeRegister(SPI_ATQ_CTRL7, newValue);
  gDeviceSpiReg.atq_ctrl7_reg = newValue;
}

/*!
 *  @brief  Programs the lower limit as % of the auto torque hysteretic band via ATQ_LL
 */
void DRV8462::ATQ_SetATQCountLowerLimitAsPercent(uint8_t newValue0to100)
{
  newValue0to100 = constrain(newValue0to100, 0, 100);
  uint8_t newValue0to255 = map(newValue0to100, 0, 100, 0, 255);
  ATQ_SetATQCountLowerLimit(newValue0to255);
}

/*!
 *  @brief  Read ATQ_LL
 */
uint8_t DRV8462::ATQ_ReadATQCountLowerLimit()
{
  uint8_t value = readRegister(SPI_ATQ_CTRL7);
  gDeviceSpiReg.atq_ctrl7_reg = value;
  return value;
}

/*!
 *  @brief  Programs the proportional constant for tuning the auto torque PD control loop (defaults to 0)
 */
void DRV8462::ATQ_SetKP(uint8_t newValue)
{
  writeRegister(SPI_ATQ_CTRL8, newValue);
  gDeviceSpiReg.atq_ctrl8_reg = newValue;
}

/*!
 *  @brief  Programs the proportional constant as % for tuning the auto torque PD control loop (defaults to 0)
 */
void DRV8462::ATQ_SetKP_AsPercent(uint8_t newValue0to100)
{
  newValue0to100 = constrain(newValue0to100, 0, 100);
  uint8_t newValue0to255 = map(newValue0to100, 0, 100, 0, 255);
  ATQ_SetKP(newValue0to255);
}

/*!
 *  @brief Programs KD 0-15 (defaults to 0)
 */
void DRV8462::ATQ_SetKD(uint8_t newValue)
{
  newValue = constrain(newValue,0,15);

  writeRegister(SPI_ATQ_CTRL9, newValue);
  gDeviceSpiReg.atq_ctrl9_reg = newValue;
}

/*!
 *  @brief Programs ATQ_FRZ 1-7 (defaults to 1)
 */
void DRV8462::ATQ_SetATQ_FRZ(uint8_t newValue)
{
  newValue = constrain(newValue,1,7);

  uint8_t registerNow = readRegister(SPI_ATQ_CTRL10);
  uint8_t ATQ_only = ((registerNow & ~ATQ_FRZ_MASK) | (newValue << 3));

  writeRegister(SPI_ATQ_CTRL10, ATQ_only);
  gDeviceSpiReg.atq_ctrl10_reg = newValue;
}




/*!
 *  @brief  Outputs the value of motor current when auto-torque is enabled. ATQ_TRQ_DAC can vary between ATQ_TRQ_MIN and ATQ_TRQ_MAX
 */
uint8_t DRV8462::ATQ_ReadTRQ_DAC()
{
  uint8_t value = readRegister(SPI_ATQ_CTRL16);
  gDeviceSpiReg.atq_ctrl16_reg = value;
  return value;
}

/*!
 *  @brief  Outputs the % value of motor current when auto-torque is enabled. 
 */
uint8_t DRV8462::ATQ_ReadTRQ_DAC_AsPercent()
{
  uint8_t value = ATQ_ReadTRQ_DAC();
  value = map(value,0,255,0,100);
  return value;
}

/*!
 *  @brief  Outputs the value of motor current when auto-torque is enabled. ATQ_TRQ_DAC can vary between ATQ_TRQ_MIN and ATQ_TRQ_MAX
 */
uint16_t DRV8462::ATQ_ReadATQ_CNT()
{
  uint8_t value10to8 = (readRegister(SPI_ATQ_CTRL2) >> 5);
  uint8_t value7to1  = readRegister(SPI_ATQ_CTRL1);
  uint16_t fullValue = (value10to8 << 8) | value7to1;
  
  // gDeviceSpiReg.atq_ctrl12_reg = fullValue; // figure out updating those 3 bits later
  gDeviceSpiReg.atq_ctrl11_reg = value7to1;
  return fullValue;
}

/*!
 *  @brief  Outputs the value of motor current when auto-torque is enabled. ATQ_TRQ_DAC can vary between ATQ_TRQ_MIN and ATQ_TRQ_MAX
 */
float DRV8462::ATQ_ReadATQ_CNTAsPercent()
{
  uint16_t fullValue = ATQ_ReadATQ_CNT();
  float percentValue = (float)fullValue / 2048.0;
  
  return percentValue;
}

/*!
 *  @brief  Outputs the value of motor current when auto-torque is enabled. ATQ_TRQ_DAC can vary between ATQ_TRQ_MIN and ATQ_TRQ_MAX
 */
uint16_t DRV8462::ATQ_ReadATQ_LRN_CONST1()
{
  uint8_t value10to8 = (readRegister(SPI_ATQ_CTRL2) & 0b00000111);
  uint8_t value7to1  = readRegister(SPI_ATQ_CTRL3);
  uint16_t fullValue = (value10to8 << 8) | value7to1;
  
  // gDeviceSpiReg.asdfasdf = fullValue; // figure out updating those 3 bits later
  gDeviceSpiReg.atq_ctrl13_reg = value7to1;
  return fullValue;
}

/*!
 *  @brief  Writes the value of ATQ_LRN_CONST1 and CONST2 to their registers
 */
void DRV8462::ATQ_SetATQ_LRN_CONST1and2(uint16_t CONST1, uint16_t CONST2)
{
  uint8_t const1_10to8 = CONST1 >> 8;
  uint8_t const1_7to1  = CONST1 & 0xFF;
  uint8_t const2_10to8 = CONST2 >> 8;
  uint8_t const2_7to1  = CONST2 & 0xFF;

  // Update CTRL2:  [7/6/5 are ATQ_CNT] / [4/3 are RSVD] / [2/1/0 are ATQ_LRN_CONST1[10:8]]
  writeRegister(SPI_ATQ_CTRL2, const1_10to8);

  // Update CTRL3: 
  writeRegister(SPI_ATQ_CTRL3, const1_7to1);

  // Update CTRL4:
  uint8_t CTRL4_value = (readRegister(SPI_ATQ_CTRL4) & 0b11111000) | const2_10to8;
  writeRegister(SPI_ATQ_CTRL4, CTRL4_value);

  // Update CTRL5:
  writeRegister(SPI_ATQ_CTRL5, const2_7to1);
}

/*!
 *  @brief  Outputs the value of motor current when auto-torque is enabled. ATQ_TRQ_DAC can vary between ATQ_TRQ_MIN and ATQ_TRQ_MAX
 */
uint16_t DRV8462::ATQ_ReadATQ_LRN_CONST2()
{
  uint8_t value10to8 = (readRegister(SPI_ATQ_CTRL4) & 0b00000111);
  uint8_t value7to1  = readRegister(SPI_ATQ_CTRL5);
  uint16_t fullValue = (value10to8 << 8) | value7to1;
  
  // gDeviceSpiReg.asdfasdf = fullValue; // figure out updating those 3 bits later
  gDeviceSpiReg.atq_ctrl15_reg = value7to1;
  return fullValue;
}

// ################################## END AUTO TORQUE #######################


// Direct reading of a register
uint8_t DRV8462::readRegister(uint8_t reg)
{
  uint8_t value = read8(reg);
  return value;
}

// Direct writing of a register
void DRV8462::writeRegister(uint8_t reg, uint8_t value)
{
  write8(reg, value);
}


/*********************************************************************/

uint8_t DRV8462::SPIxfer(uint8_t x) {
  if (_clk == -1) {
    return _spi->transfer(x);
  } else {
    // Serial.println("Software SPI");
    uint8_t reply = 0;
    for (int i = 7; i >= 0; i--) {
      reply <<= 1;
      digitalWrite(_clk, HIGH);
      digitalWrite(_mosi, x & (1 << i));
      digitalWrite(_clk, LOW);
      if (digitalRead(_miso))
        reply |= 1;
    }
    return reply;
  }
}

uint8_t DRV8462::read8(uint8_t reg) {

  // This SPI function is used to read the device configurations, parameters and
  // status information for S version of device.
  // Register format |R/W|A5|A4|A3|A2|A1|A0|*|D7|D6|D5|D4|D3|D2|D1|D0|
  // Ax is address bit, Dx is data bits and R/W is read write bit.
  // For read R/W bit should be 1.
  volatile uint16_t reg_value = 0;
  reg_value |= ((reg << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         // Configure register address value
  reg_value |= SPI_RW_BIT_MASK;                                           // Set R/W bit

  uint8_t value;

  digitalWrite(_cs, LOW);
  SPIxfer((uint8_t)((reg_value >>8) & 0xFF));
  value = SPIxfer(0x00);
  digitalWrite(_cs, HIGH);

  return value;
}

void DRV8462::write8(uint8_t reg, uint8_t value) {

  // This SPI function is used to write the set device configurations and operating
  // parameters of the device.
  // Register format |R/W|A5|A4|A3|A2|A1|A0|*|D7|D6|D5|D4|D3|D2|D1|D0|
  // Ax is address bit, Dx is data bits and R/W is read write bit.
  // For write R/W bit should 0.

  volatile uint16_t reg_value = 0; // Variable for the combined register and data info
  reg_value |= ((reg << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         // Adding register address value
  reg_value |= ((value << SPI_DATA_POS) & SPI_DATA_MASK);             // Adding data value

  digitalWrite(_cs, LOW);

  SPIxfer((uint8_t)((reg_value>>8) & 0xFF));
  SPIxfer((uint8_t)(reg_value & 0xFF));

  digitalWrite(_cs, HIGH);
}