/*!
 * @file DRV8462.h
 */

#ifndef __DRV8462_H__
#define __DRV8462_H__

#include <Arduino.h>
#include <SPI.h>

// SPI Protocol
#define SPI_ADDRESS_MASK   0x3F00        // Mask for SPI register address bits
#define SPI_ADDRESS_POS    8             // Position for SPI register address bits
#define SPI_DATA_MASK      0x00FF        // Mask for SPI register data bits
#define SPI_DATA_POS       0             // Position for SPI register data bits
#define SPI_RW_BIT_MASK    0x4000        // Mask for SPI register read write indication bit

// Bit Mask definitions for Registers
// Register 0x00: FAULT Register
#define OL_MASK               (0x01)         // Indicates open load condition
#define OT_MASK               (0x02)         // Logic OR of OTW and OTSD
#define STL_MASK              (0x04)         // Indicates motor stall
#define OCP_MASK              (0x08)         // Indicates over current fault condition
#define CPUV_MASK             (0x10)         // Indicates charge pump under voltage fault condition
#define UVLO_MASK             (0x20)         // Indicates an supply under voltage lockout fault condition
#define SPI_ERROR_MASK        (0x40)         // Indication SPI communication error
#define FAULT_MASK            (0x80)         // Inverse value of the nFAULT pin

// Register 0x01 : DIAG1 Register
#define OCP_HS1_A_MASK        (0x01)        // Indicates over current fault on the high-side FET of AOUT1
#define OCP_LS1_A_MASK        (0x02)        // Indicates over current fault on the low-side FET of AOUT1
#define OCP_HS2_A_MASK        (0x04)        // Indicates over current fault on the high-side FET of AOUT2
#define OCP_LS2_A_MASK        (0x08)        // Indicates over current fault on the low-side FET of AOUT2
#define OCP_HS1_B_MASK        (0x10)        // Indicates over current fault on the high-side FET of BOUT1
#define OCP_LS1_B_MASK        (0x20)        // Indicates over current fault on the low-side FET of BOUT1
#define OCP_HS2_B_MASK        (0x40)        // Indicates over current fault on the high-side FET of BOUT2
#define OCP_LS2_B_MASK        (0x80)        // Indicates over current fault on the low-side FET of BOUT2

// Register 0x02 : DIAG2 Register
#define OL_A_MASK             (0x01)        // Indicates open-load detection on AOUT
#define OL_B_MASK             (0x02)        // Indicates open-load detection on BOUT
#define ATQ_LRN_DONE_MASK     (0x04)        // Indicates auto torque learning was successful
#define STALL_MASK            (0x08)        // Indicates motor stall condition
#define STL_LRN_OK_MASK       (0x10)        // Indicates stall detection learning was successful
#define OTS_MASK              (0x20)        // Indicates over temperature shutdown
#define OTW_MASK              (0x40)        // Indicates over temperature warning
#define STSL_MASK             (0x80)        // Indicates motor standstill

// Register 0x03 : DIAG3 Register
#define RSVD0_MASK            (0x01)        // Reserved bit
#define RSVD1_MASK            (0x02)        // Reserved bit
#define NPOR_MASK             (0x04)        // Indicates a VCC UVLO event
#define SILENTSTEP_ERROR_MASK (0x08)        // Indicates Silent Step operation error
#define ATQ_CNT_UFLW_MASK     (0x10)        // Indicates ATQ_CNT is less than ATQ_LL
#define ATQ_CNT_OFLW_MASK     (0x20)        // Indicates ATQ_CNT is more than ATQ_UL
#define NHOME_MASK            (0x40)        // Indicates indexer is at home position of step table
#define SILENTSTEP_ON_MASK    (0x80)        // Indicates that device is working with silentstep decay mode

// Register 0x04 : CTRL1 Register
#define DECAY_MASK            (0x07)        // Bridge decay setting
#define TOFF_MASK             (0x18)        // Current regulation TOFF setting
#define RSVD2_MASK            (0x20)        // Reserved bit
#define SR_MASK               (0x40)        // Output driver rise and fall time selection
#define EN_OUT_MASK           (0x80)        // Hi-Z outputs bit OR-ed with DRVOFF

// Register 0x05 : CTRL2 Register
#define MICROSTEP_MODE_MASK   (0x0F)        // Microstep setting
#define SPI_STEP_MASK         (0x10)        // Enable SPI step control mode
#define SPI_DIR_MASK          (0x20)        // Enable SPI direction control mode
#define STEP_MASK             (0x40)        // Step control bit if SPI_STEP is enabled
#define DIR_MASK              (0x80)        // Direction control bit if SPI_DIR is enabled

// Register 0x06 : CTRL3 Register
#define TW_REP_MASK           (0x01)        // Report OTW on nFAULT
#define OTSD_MODE_MASK        (0x02)        // OTSD latch fault setting
#define OCP_MODE_MASK         (0x04)        // OCP latch fault setting
#define TOCP_MASK             (0x08)        // OCP deglitch time setting
#define LOCK_MASK             (0x70)        // Lock SPI registers
#define CLR_FLT_MASK          (0x80)        // Clear all fault bits

// Register 0x07 : CTRL4 Register
#define STEP_FREQ_TOL_MASK    (0x03)        // Programs the filter setting on the STEP frequency input.
#define RSVD3_MASK            (0x04)        // Reserved bit
#define STL_REP_MASK          (0x08)        // Report stall detection on nFAULT
#define EN_STL_MASK           (0x10)        // Enable stall detection
#define DIS_STL_MASK          (0xEF)        // Disable stall detection - clears EN_STL bit
#define STL_LRN_MASK          (0x20)        // Learn stall count threshold
#define TBLANK_TIME_MASK      (0xC0)        // Controls the current sense blanking time

// Register 0x08 : CTRL5 Register
#define STALL_TH_MASK         (0xFF)        // Stall Threshold Lower 8-bits

// Register 0x09 : CTRL6 Register
#define STALL_TH_MSB_MASK     (0x0F)        // Stall Threshold Upper 4-bits
#define TRQ_SCALE_MASK        (0x80)        // Torque scaling 0: x1 and 1: x8
#define DIS_SSC_MASK          (0x40)        // Spread-spectrum disable for CP and OSC
#define RC_RIPPLE             (0xC0)        // Controls the current ripple in smart tune ripple control decay mode

// Register 0x0A : CTRL7 Register
#define TRQ_COUNT_MASK        (0xFF)        // Torque Count Lower 8-bits

// Register 0x0B : CTRL8 Register
#define TRQ_COUNT_MASK_MSB    (0x0F)        // Torque Count Upper 4-bits
#define REV_ID_MASK           (0xF0)        // Silicon Revision ID

// Register 0x0C : CTRL9 Register
#define EN_AUTO_MASK           (0x01)        // Automatic Microstepping bit
#define RES_AUTO_MASK          (0b00000110)  // Automatic Microstepping Resolution - default of 1/256

// Register 0x0F : CTRL12 Register
#define EN_STSL_MASK          (0x80)        // Standstill power saving mode bit

// Register 0x10 : CTRL13 Register
#define TSTSL_DLY_MASK        (0b11111100)        // Controls the delay between last STEP pulse and activation of standstill power saving mode. 16ms to 1008ms
#define VREF_INT_EN_MASK      (0x02)        // Use the internal 3.3V reference for current regulation.


// Register ATQ_CTRL1 Auto Torque is just ATQ_CNT[7:0]

// Register ATQ_CTRL2 Auto Torque
#define ATQ_CNT_MASK           (0b11100000)  // ATQ_CNT[10:8]
#define ATQ_LRN_CONST_MASK     (0b00000111)  // ATQ_LRN_CONST[10:8]

// Register ATQ_CTRL3 Auto Torque is just ATQ_LRN_CONST[7:0]

// Register ATQ_CTRL4 Auto Torque
#define ATQ_LRN_MIN_CURRENT_MASK (0b11111000)
#define ATQ_LRN_CONST2_MASK      (0b00000111) //  ATQ_LRN_CONST2[10:8]

// Register ATQ_CTRL5 Auto Torque is just ATQ_LRN_CONST2[7:0]

// Register ATQ_CTRL6 Auto Torque is just ATQ_UL

// Register ATQ_CTRL7 Auto Torque is just ATQ_LL

// Register ATQ_CTRL8 Auto Torque is just ATQ_KP

// Register ATQ_CTRL9 Auto Torque is just ATQ_KD

// Register ATQ_CTRL10 Auto Torque
#define ATQ_EN_MASK           (0b10000000)
#define LRN_START_MASK        (0b01000000)
#define ATQ_FRZ_MASK          (0b00111000)
#define ATQ_AVG_MASK          (0b00000111)

// Register ATQ_CTRL11 Auto Torque is just ATQ_TRQ_MIN

// Register ATQ_CTRL12 Auto Torque is just ATQ_TRQ_MAX

// Register ATQ_CTRL13 Auto Torque is just ATQ_D_THR

// Register ATQ_CTRL14 Auto Torque is RSVD

// Register ATQ_CTRL15 Auto Torque
#define ATQ_ERROR_TRUNCATE_MASK   (0b11110000)
#define ATQ_LRN_STEP_MASK         (0b00001100)
#define ATQ_LRN_CYCLE_SELECT_MASK (0b00000011)

// Register ATQ_CTRL16 Auto Torque is just ATQ_TRQ_DAC

// Register ATQ_CTRL17 Auto Torque
#define ATQ_VM_SCALE_MASK   (0b01000000)

// Register ATQ_CTRL18 Auto Torque is RSVD

/// DRV8xx2 SPI Register Addresses
enum
{
  SPI_FAULT             = 0x00,
  SPI_DIAG1             = 0x01,
  SPI_DIAG2             = 0x02,
  SPI_DIAG3             = 0x03,
  SPI_CTRL1             = 0x04,
  SPI_CTRL2             = 0x05,
  SPI_CTRL3             = 0x06,
  SPI_CTRL4             = 0x07,
  SPI_CTRL5             = 0x08,
  SPI_CTRL6             = 0x09,
  SPI_CTRL7             = 0x0A,
  SPI_CTRL8             = 0x0B,
  SPI_CTRL9             = 0x0C,
  SPI_CTRL10            = 0x0D,
  SPI_CTRL11            = 0x0E,
  SPI_CTRL12            = 0x0F,
  SPI_CTRL13            = 0x10,
  SPI_INDEX1            = 0x11,
  SPI_INDEX2            = 0x12,
  SPI_INDEX3            = 0x13,
  SPI_INDEX4            = 0x14,
  SPI_INDEX5            = 0x15,
  SPI_CUSTOM_CTRL1      = 0x16,
  SPI_CUSTOM_CTRL2      = 0x17,
  SPI_CUSTOM_CTRL3      = 0x18,
  SPI_CUSTOM_CTRL4      = 0x19,
  SPI_CUSTOM_CTRL5      = 0x1A,
  SPI_CUSTOM_CTRL6      = 0x1B,
  SPI_CUSTOM_CTRL7      = 0x1C,
  SPI_CUSTOM_CTRL8      = 0x1D,
  SPI_CUSTOM_CTRL9      = 0x1E,
  SPI_ATQ_CTRL1         = 0x1F,
  SPI_ATQ_CTRL2         = 0x20,
  SPI_ATQ_CTRL3         = 0x21,
  SPI_ATQ_CTRL4         = 0x22,
  SPI_ATQ_CTRL5         = 0x23,
  SPI_ATQ_CTRL6         = 0x24,
  SPI_ATQ_CTRL7         = 0x25,
  SPI_ATQ_CTRL8         = 0x26,
  SPI_ATQ_CTRL9         = 0x27,
  SPI_ATQ_CTRL10        = 0x28,
  SPI_ATQ_CTRL11        = 0x29,
  SPI_ATQ_CTRL12        = 0x2A,
  SPI_ATQ_CTRL13        = 0x2B,
  SPI_ATQ_CTRL14        = 0x2C,
  SPI_ATQ_CTRL15        = 0x2D,
  SPI_ATQ_CTRL16        = 0x2E,
  SPI_ATQ_CTRL17        = 0x2F,
  SPI_ATQ_CTRL18        = 0x30,
  SPI_SILENTSTEP_CTRL1  = 0x31,
  SPI_SILENTSTEP_CTRL2  = 0x32,
  SPI_SILENTSTEP_CTRL3  = 0x33,
  SPI_SILENTSTEP_CTRL4  = 0x34,
  SPI_SILENTSTEP_CTRL5  = 0x35,
  SPI_CTRL14            = 0x3C,
};

/// Possible arguments to setStepMode().
typedef enum
{
  /// Full step with 100% current
  MicroStep1_100 = 0b0000,

  /// Full step with 71% current
  MicroStep1     = 0b0001,

  /// Non-circular 1/2 step
  MicroStep2_NC  = 0b0010,

  /// Circular 1/2 step
  MicroStep2     = 0b0011,

  MicroStep4     = 0b0100,
  MicroStep8     = 0b0101,
  MicroStep16    = 0b0110,
  MicroStep32    = 0b0111,
  MicroStep64    = 0b1000,
  MicroStep128   = 0b1001,
  MicroStep256   = 0b1010,
} DRV8462_microstep_t;

/// Possible arguments to setDecayMode().
typedef enum
{
  SlowDecay = 0b000,
  Mixed30Decay = 0b100,
  Mixed60Decay = 0b101,
  SmartTuneDynamicDecay = 0b110,
  SmartTuneRippleControl = 0b111,

} DRV8462_decay_t;

// Structure variables for the DRV8xx2 registers
typedef struct DRV8xx2_REG
{
    uint8_t fault_status_reg = 0;
    uint8_t diag_status1_reg = 0;
    uint8_t diag_status2_reg = 0;
    uint8_t diag_status3_reg = 0;
    uint8_t ctrl1_reg = 0x0F;
    uint8_t ctrl2_reg = 0x06;
    uint8_t ctrl3_reg = 0x38;
    uint8_t ctrl4_reg = 0x08;
    uint8_t ctrl5_reg = 0x03;
    uint8_t ctrl6_reg = 0x00;
    uint8_t ctrl7_reg = 0xFF;
    uint8_t ctrl8_reg = 0x0F;
    uint8_t ctrl9_reg = 0x00;
    uint8_t ctrl10_reg = 0x80;
    uint8_t ctrl11_reg = 0xFF;
    uint8_t ctrl12_reg = 0x20;
    uint8_t ctrl13_reg = 0x10;
    uint8_t ctrl14_reg = 0x58;
    uint8_t index1_reg = 0x80;
    uint8_t index2_reg = 0x80;
    uint8_t index3_reg = 0x80;
    uint8_t index4_reg = 0x82;
    uint8_t index5_reg = 0xB5;
    uint8_t custom_ctrl1_reg = 0x00;
    uint8_t custom_ctrl2_reg = 0x00;
    uint8_t custom_ctrl3_reg = 0x00;
    uint8_t custom_ctrl4_reg = 0x00;
    uint8_t custom_ctrl5_reg = 0x00;
    uint8_t custom_ctrl6_reg = 0x00;
    uint8_t custom_ctrl7_reg = 0x00;
    uint8_t custom_ctrl8_reg = 0x00;
    uint8_t custom_ctrl9_reg = 0x00;
    uint8_t atq_ctrl1_reg = 0x00;
    uint8_t atq_ctrl2_reg = 0x00;
    uint8_t atq_ctrl3_reg = 0x00;
    uint8_t atq_ctrl4_reg = 0x20;
    uint8_t atq_ctrl5_reg = 0x00;
    uint8_t atq_ctrl6_reg = 0x00;
    uint8_t atq_ctrl7_reg = 0x00;
    uint8_t atq_ctrl8_reg = 0x00;
    uint8_t atq_ctrl9_reg = 0x00;
    uint8_t atq_ctrl10_reg = 0x08;
    uint8_t atq_ctrl11_reg = 0x0A;
    uint8_t atq_ctrl12_reg = 0xFF;
    uint8_t atq_ctrl13_reg = 0x05;
    uint8_t atq_ctrl14_reg = 0x0F;
    uint8_t atq_ctrl15_reg = 0x00;
    uint8_t atq_ctrl16_reg = 0xFF;
    uint8_t atq_ctrl17_reg = 0x00;
    uint8_t atq_ctrl18_reg = 0x00;
    uint8_t silentstep_ctrl1_reg = 0x00;
    uint8_t silentstep_ctrl2_reg = 0x00;
    uint8_t silentstep_ctrl3_reg = 0x00;
    uint8_t silentstep_ctrl4_reg = 0x00;
    uint8_t silentstep_ctrl5_reg = 0xFF;
} DRV8xx2_REG_t;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          DRV8462
 */
class DRV8462 {
public:

  DRV8462(int8_t SPICLK = 18, int8_t SPIMISO = 19, int8_t SPIMOSI = 23, int8_t SPICS = 5);

  boolean beginSPI ();
  void setPins(int8_t enablePin = 2, int8_t nSleepPin = 4, int8_t nFaultPin = 15, int8_t stepPin = 16, int8_t dirPin = 17);

  void FullInitialize(int8_t enablePin = 2, int8_t nSleepPin = 4, int8_t nFaultPin = 15, int8_t stepPin = 16, int8_t dirPin = 17 );

  void enableOutputs();
  void disableOutputs();

  void setDirectionForward();
  void setDirectionReverse();
  void toggleDirection();

  void wakeup_nSLEEP_HIGH();
  void asleep_nSLEEP_LOW();

  void clearFaultPulse();
  void clearFaultSPI();
  void clearFaults();

  void setTorqueDAC(uint8_t newDACValue0to255);
  void setTorqueDACPercent(uint8_t newDACValue0to100);
  void setRunCurrentPercent(uint8_t newDACValue0to100);

  void enableAutoMicrostepping();
  void disableAutoMicrostepping();
  void setAutoMicroResolution(uint16_t modeNum = 0);

  void setOutputSlewRate(uint8_t modeNum);
  void setTOFF(uint8_t modeNum);

  void setOCP_MODE_autoRecovery();
  void setOCP_MODE_latchedFault();

  void enableStandstillPowerSaving();
  void disableStandstillPowerSaving();
  void setHoldingCurrent(uint8_t newHoldingCurrent);
  void setHoldingCurrentPercent(uint8_t newHoldingCurrentPercent);
  void setStandstillDelayTime(uint16_t newDelayTime);
  void setStandstillDelayTimeMS(uint16_t newDelayTimeMilliseconds);

  void setTBLANKTime_us(uint8_t newDelayTime_us);

  void setMicrostepMode(DRV8462_microstep_t microstep = MicroStep16);
  void setMicrostepMode(uint16_t modeNum);

  void setDecayMode(DRV8462_decay_t decayMode = SmartTuneRippleControl);
  
  void enableAutoTorqueMode();
  void disableAutoTorqueMode();
  void ATQ_StartLearning();
  uint8_t ATQ_ReadLRN_START();
  uint8_t ATQ_ReadLRN_DONE();
  void ATQ_enableVMScaling();
  void ATQ_disableVMScaling();
  void ATQ_SetInitialLearnCurrent(uint8_t newValue);
  void ATQ_SetCurrentStepsForLearn(uint8_t newValue);
  void ATQ_SetCurrentCyclesForLearn(uint8_t newValue);
  void ATQ_SetMaxCurrentLimit(uint8_t newValue);
  void ATQ_SetMaxCurrentLimitAsPercent(uint8_t newValue0to100);
  void ATQ_SetMinCurrentLimit(uint8_t newValue);
  void ATQ_SetMinCurrentLimitAsPercent(uint8_t newValue0to100);
  uint8_t ATQ_ReadATQCountUpperLimit();
  uint8_t ATQ_ReadATQCountLowerLimit();
  uint8_t ATQ_ReadMinCurrentLimit();
  uint8_t ATQ_ReadMaxCurrentLimit();
  void ATQ_SetATQCountUpperLimit(uint8_t newValue);
  void ATQ_SetATQCountUpperLimitAsPercent(uint8_t newValue0to100);
  void ATQ_SetATQCountLowerLimit(uint8_t newValue);
  void ATQ_SetATQCountLowerLimitAsPercent(uint8_t newValue0to100);
  void ATQ_SetKP(uint8_t newValue);
  void ATQ_SetKP_AsPercent(uint8_t newValue0to100);
  void ATQ_SetKD(uint8_t newValue);
  void ATQ_SetATQ_FRZ(uint8_t newValue);
  uint8_t ATQ_ReadTRQ_DAC();
  uint8_t ATQ_ReadTRQ_DAC_AsPercent();
  uint16_t ATQ_ReadATQ_CNT();
  float ATQ_ReadATQ_CNTAsPercent();
  uint16_t ATQ_ReadATQ_LRN_CONST1();
  uint16_t ATQ_ReadATQ_LRN_CONST2();
  void ATQ_SetATQ_LRN_CONST1and2(uint16_t CONST1, uint16_t CONST2);


  SPIClass *_spi; //!< pointer to SPI object
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);

private:
  uint8_t SPIxfer(uint8_t x);
  uint8_t read8(uint8_t addr);
  uint16_t read16(uint8_t addr);
  void write8(uint8_t addr, uint8_t data);

  // Pin Assignments
  int8_t _cs, _clk, _miso, _mosi;
  int8_t _enable, _nsleep, _nfault, _step, _dir;
};


// /// This class provides low-level functions for reading and writing from the SPI
// /// interface of a DRV8462 stepper motor controller IC.
// ///
// /// Most users should use the HighPowerStepperDriver class, which provides a
// /// higher-level interface, instead of this class.  
// // https://github.com/pololu/high-power-stepper-driver-arduino/blob/master/HighPowerStepperDriver.h
// class DRV8462SPI
// {
// public:
//   /// Configures this object to use the specified pin as a chip select pin.
//   ///
//   /// You must use a chip select pin; the DRV8434 requires it.
//   void setChipSelectPin(uint8_t pin)
//   {
//     csPin = pin;
//     pinMode(csPin, OUTPUT);
//     digitalWrite(csPin, HIGH);
//   }

//   /// Reads the register at the given address and returns its raw value.
//   uint8_t readReg(SPIClass *spi, uint8_t address)
//   {
//     // This SPI function is used to read the device configurations, parameters and
//     // status information for S version of device.
//     // Register format |R/W|A5|A4|A3|A2|A1|A0|*|D7|D6|D5|D4|D3|D2|D1|D0|
//     // Ax is address bit, Dx is data bits and R/W is read write bit.
//     // For read R/W bit should be 1.
//     volatile uint16_t reg_value = 0;
//     volatile uint8_t dataMSB    = 0;
//     volatile uint8_t dataLSB    = 0;

//     reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         // Configure register address value
//     reg_value |= SPI_RW_BIT_MASK;                                           // Set R/W bit

//     selectChip(spi);
//     uint8_t temp1 = transfer((uint8_t)((reg_value >>8) & 0xFF));               // Transmit the Address(MSB Byte)
//     // lastStatus = transfer((0x20 | (address & 0b11111)) << 1);
//     uint8_t data = transfer(0);
//     deselectChip(spi);
//     return data;
//   }

//   /// Reads the register at the given address and returns its raw value.
//   uint16_t readReg(DRV8462RegAddr address)
//   {
//     return readReg((uint8_t)address);
//   }

//   /// Writes the specified value to a register.
//   uint8_t writeReg(uint8_t address, uint8_t value)
//   {
//     // This SPI function is used to write the set device configurations and operating
//     // parameters of the device.
//     // Register format |R/W|A5|A4|A3|A2|A1|A0|*|D7|D6|D5|D4|D3|D2|D1|D0|
//     // Ax is address bit, Dx is data bits and R/W is read write bit.
//     // For write R/W bit should 0.

//     volatile uint16_t reg_value = 0;

//     reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         // Adding register address value
//     reg_value |= ((value << SPI_DATA_POS) & SPI_DATA_MASK);                 // Adding data value

//     selectChip(spi);
//     // lastStatus = transfer((address & 0b11111) << 1);
//     uint8_t temp1 = transfer((uint8_t)((reg_value>>8) & 0xFF));                // Transmit first Byte,MSB-Byte
//     // uint8_t oldData = transfer(value);
//     uint8_t oldData = transfer((uint8_t)(reg_value & 0xFF));                // Transmit Second Byte, LSB-Byte
//     // The CS line must go low after writing for the value to actually take effect.
//     deselectChip(spi);
//     return oldData;
//   }

//   /// Writes the specified value to a register.
//   void writeReg(SPIClass *spi, DRV8462RegAddr address, uint8_t value)
//   {
//     writeReg(spi, (uint8_t)address, value);
//   }
//   SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE1);

//   uint8_t transfer(SPIClass *spi, uint8_t value)
//   {
//     return spi->transfer(value);
//   }

//   void selectChip(SPIClass *spi)
//   {
//     digitalWrite(csPin, LOW);
//     spi->beginTransaction(settings);
//   }

//   void deselectChip(SPIClass *spi)
//   {
//    spi->endTransaction();
//    digitalWrite(csPin, HIGH);
//   }

//   uint8_t csPin;

//   /// The status reported by the driver during the last read or write.  This
//   /// status is the same as that which would be returned by reading the FAULT
//   /// register with DRV8462::readFault(), except the upper two bits are always
//   /// 1.
//   uint8_t lastStatus = 0;
  

// // private:

  

// // public:

  
// };


// /// Bits that are set in the return value of readFault() to indicate warning and
// /// fault conditions.
// ///
// /// See the DRV8462 datasheet for detailed descriptions of these conditions.
// enum class DRV8462FaultBit : uint8_t
// {
//   /// Inverse value of the nFAULT pin.  Fault indication (0 when nFAULT pin is high, 1 when nFAULT pin is low)
//   FAULT = 7,

//   /// Indication SPI communication error (latched)
//   SPI_ERROR = 6,

//   /// Indicates an supply under voltage lockout fault condition
//   UVLO = 5,

//   /// Indicates charge pump under voltage fault condition
//   CPUV = 4,

//   /// Overcurrent fault
//   OCP = 3,

//   /// Motor stall
//   STL = 2,

//   /// Overtemperature warning or shutdown.  Logic OR of OTW and OTSD
//   TF = 1,

//   /// Open load - Indicates open load condition
//   OL = 0,
// };

// /// Bits that are set in the return value of readDiag1() to indicate warning and
// /// fault conditions.
// ///
// /// See the DRV8462 datasheet for detailed descriptions of these conditions.
// enum class DRV8462Diag1Bit : uint8_t
// {
//   /// Indicates over current fault on the low-side FET of BOUT2
//   OCP_LS2_B = 7,

//   /// Indicates over current fault on the high-side FET of BOUT2
//   OCP_HS2_B = 6,

//   /// Indicates over current fault on the low-side FET of BOUT1
//   OCP_LS1_B = 5,

//   /// Indicates over current fault on the high-side FET of BOUT1
//   OCP_HS1_B = 4,

//   /// Indicates over current fault on the low-side FET of AOUT2
//   OCP_LS2_A = 3,

//   /// Indicates over current fault on the high-side FET of AOUT2
//   OCP_HS2_A = 2,

//   /// Indicates over current fault on the low-side FET of AOUT1
//   OCP_LS1_A = 1,

//   /// Indicates over current fault on the high-side FET of AOUT1
//   OCP_HS1_A = 0,
// };

// /// Bits that are set in the return value of readDiag2() to indicate warning and
// /// fault conditions.
// ///
// /// See the DRV8462 datasheet for detailed descriptions of these conditions.
// enum class DRV8462Diag2Bit : uint8_t
// {
//   /// Indicates motor standstill
//   STSL= 7,

//   /// Overtemperature warning
//   OTW = 6,

//   /// Overtemperature shutdown
//   OTS = 5,

//   /// Indicates stall detection learning was successful
//   STL_LRN_OK = 4,

//   /// Indicates motor stall condition
//   STALL = 3,

//   /// Indicates auto torque learning was successful
//   ATQ_LRN_DONE = 2,

//   /// Indicates open-load detection on BOUT
//   OL_B = 1,

//   /// Indicates open-load detection on AOUT
//   OL_A = 0,
// };

// /// Bits that are set in the return value of readDiag3() to indicate warning and fault conditions.
// ///
// /// See the DRV8462 datasheet for detailed descriptions of these conditions.
// enum class DRV8462Diag3Bit : uint8_t
// {
//   /// Indicates that device is working with silentstep decay mode
//   SILENTSTEP_ON = 7,

//   /// Indicates indexer is at home position of step table
//   NHOME = 6,

//   /// Indicates ATQ_CNT is more than ATQ_UL
//   ATQ_CNT_OFLW = 5,

//   /// Indicates ATQ_CNT is less than ATQ_LL
//   ATQ_CNT_UFLW = 4,

//   /// Indicates Silent Step operation error
//   SILENTSTEP_ERROR = 3,

//   /// Indicates a VCC UVLO event
//   NPOR = 2,

// };

// /// Possible arguments to setDecayMode().
// enum class DRV8462DecayMode : uint8_t
// {
//   Slow                   = 0b000,
//   Mixed30                = 0b100,
//   Mixed60                = 0b101,
//   SmartTuneDynamicDecay  = 0b110,

//   // Default
//   SmartTuneRippleControl = 0b111, 
// };

// /// Possible arguments to setStepMode().
// enum class DRV8462StepMode : uint8_t
// {
//   /// Full step with 100% current
//   MicroStep1_100 = 0b0000,

//   /// Full step with 71% current
//   MicroStep1     = 0b0001,

//   /// Non-circular 1/2 step
//   MicroStep2_NC  = 0b0010,

//   /// Circular 1/2 step
//   MicroStep2     = 0b0011,

//   MicroStep4     = 0b0100,
//   MicroStep8     = 0b0101,
//   MicroStep16    = 0b0110,
//   MicroStep32    = 0b0111,
//   MicroStep64    = 0b1000,
//   MicroStep128   = 0b1001,
//   MicroStep256   = 0b1010,
// };


// /// This class provides high-level functions for controlling a DRV8462 stepper
// /// motor driver.
// class DRV8462
// {
// public:
//   /// The default constructor.
//   DRV8462()
//   {
//     // All settings set to power-on defaults
//     ctrl1 = 0b00001111;
//     ctrl2 = 0b00000110;
//     ctrl3 = 0x06;
//     ctrl4 = 0x30;
//     ctrl5 = 0x08;
//     ctrl6 = 0x03;
//     ctrl7 = 0x20;
//   }

//   /// Configures this object to use the specified pin as a chip select pin.
//   /// You must use a chip select pin; the DRV8711 requires it.
//   void setChipSelectPin(uint8_t pin)
//   {
//     driver.setChipSelectPin(pin);
//   }

//   /// Changes all of the driver's settings back to their default values.
//   ///
//   /// It is good to call this near the beginning of your program to ensure that
//   /// there are no settings left over from an earlier time that might affect the
//   /// operation of the driver.
//   // void resetSettings()
//   // {
//   //   ctrl1 = 0x00;
//   //   ctrl2 = 0x0F;
//   //   ctrl3 = 0x06;
//   //   ctrl4 = 0x30;
//   //   ctrl5 = 0x08;
//   //   ctrl6 = 0x03;
//   //   ctrl7 = 0x20;
//   //   applySettings();
//   // }

//   /// Reads back the SPI configuration registers from the device and verifies
//   /// that they are equal to the cached copies stored in this class.
//   ///
//   /// This can be used to verify that the driver is powered on and has not lost
//   /// them due to a power failure.  The STATUS register is not verified because
//   /// it does not contain any driver settings.
//   ///
//   /// @return 1 if the settings from the device match the cached copies, 0 if
//   /// they do not.
//   // bool verifySettings()
//   // {
//   //   return driver.readReg(DRV8462RegAddr::SPI_CTRL1) == ctrl1 &&
//   //          driver.readReg(DRV8462RegAddr::SPI_CTRL2) == ctrl2 &&
//   //          driver.readReg(DRV8462RegAddr::SPI_CTRL3) == ctrl3 &&
//   //          driver.readReg(DRV8462RegAddr::SPI_CTRL4) == ctrl4 &&
//   //          driver.readReg(DRV8462RegAddr::SPI_CTRL5) == ctrl5 &&
//   //          driver.readReg(DRV8462RegAddr::SPI_CTRL6) == ctrl6 &&
//   //          driver.readReg(DRV8462RegAddr::SPI_CTRL7) == ctrl7;
//   // }

//   /// Re-writes the cached settings stored in this class to the device.
//   ///
//   /// You should not normally need to call this function because settings are
//   /// written to the device whenever they are changed.  However, if
//   /// verifySettings() returns false (due to a power interruption, for
//   /// instance), then you could use applySettings() to get the device's settings
//   /// back into the desired state.
//   // void applySettings()
//   // {
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL1);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL3);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL4);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL5);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL6);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL7);

//   //   // CTRL2 is written last because it contains the EN_OUT bit, and we want to
//   //   // try to have all the other settings correct first.
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL2);
//   // }

//   /// Sets the driver's current scalar (TRQ_DAC), which scales the full current
//   /// limit (as set by VREF) by the specified percentage. The available settings
//   /// are multiples of 6.25%.
//   ///
//   /// This function takes an integer, and if the desired current limit is not
//   /// available, it generally tries to pick the closest current limit that is
//   /// lower than the desired one (although the lowest possible setting is
//   /// 6.25%). However, it will round up if the next setting is no more than
//   /// 0.75% higher; this allows you to specify 43.75% by passing a value of 43,
//   /// for example.
//   ///
//   /// Example usage:
//   /// ~~~{.cpp}
//   /// // This sets TRQ_DAC to 37.5% (the closest setting lower than 42%):
//   /// sd.setCurrentPercent(42);
//   ///
//   /// // This sets TRQ_DAC to 43.75% (rounding 43 up by 0.75% to 43.75%):
//   /// sd.setCurrentPercent(43);
//   ///
//   /// // This also sets TRQ_DAC to 43.75%; even though the argument is truncated
//   /// // to an integer (43), that is then rounded up by 0.75% to 43.75%:
//   /// sd.setCurrentPercent(43.75);
//   /// ~~~
//   // void setCurrentPercent(uint8_t percent)
//   // {
//   //   if (percent > 100) { percent = 100; }
//   //   if (percent < 6) { percent = 6; }

//   //   uint8_t td = ((uint16_t)percent * 4 + 3) / 25; // convert 6-100% to 1-16, rounding up by at most 0.75%
//   //   td = 16 - td;                                  // convert 1-16 to 15-0 (15 = 6.25%, 0 = 100%)
//   //   ctrl1 = (ctrl1 & 0b00001111) | (td << 4);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL1);
//   // }

//   /// Sets the driver's current scalar (TRQ_DAC) to produce the specified scaled
//   /// current limit in milliamps. In order to calculate the correct value for
//   /// TRQ_DAC, this function also needs to know the full current limit set by
//   /// VREF (i.e. what the current limit is when the scaling is set to 100%).
//   /// This is specified by the optional `fullCurrent` argument, which defaults
//   /// to 2000 milliamps (2 A).
//   ///
//   /// If the desired current limit is not
//   /// available, this function tries to pick the closest current limit that is
//   /// lower than the desired one (although the lowest possible setting is 6.25%
//   /// of the full current limit).
//   ///
//   /// Example usage:
//   /// ~~~{.cpp}
//   /// // This specifies that we want a scaled current limit of 1200 mA and that
//   /// // VREF is set to produce a full current limit of 1500 mA. TRQ_DAC will be
//   /// // set to 75%, which will produce a 1125 mA scaled current limit.
//   /// sd.setCurrentMilliamps(1200, 1500);
//   /// ~~~
//   // void setCurrentMilliamps(uint16_t current, uint16_t fullCurrent = 2000)
//   // {
//   //   if (fullCurrent > 4000) { fullCurrent = 4000; }
//   //   if (current > fullCurrent) { current = fullCurrent; }

//   //   uint8_t td = (current * 16 / fullCurrent); // convert 0-fullCurrent to 0-16
//   //   if (td == 0) { td = 1; }                   // restrict to 1-16
//   //   td = 16 - td;                              // convert 1-16 to 15-0 (15 = 6.25%, 0 = 100%)
//   //   ctrl1 = (ctrl1 & 0b00001111) | (td << 4);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL1);
//   // }

//   // /// Enables the driver (EN_OUT = 1).
//   // void enableDriver()
//   // {
//   //   ctrl2 |= (1 << 7);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL2);
//   // }

//   // /// Disables the driver (EN_OUT = 0).
//   // void disableDriver()
//   // {
//   //   ctrl2 &= ~(1 << 7);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL2);
//   // }

//   /// Sets the driver's decay mode (DECAY).
//   ///
//   /// Example usage:
//   /// ~~~{.cpp}
//   /// sd.setDecayMode(DRV8462DecayMode::SmartTuneDynamicDecay);
//   /// ~~~
//   // void setDecayMode(DRV8462DecayMode mode)
//   // {
//   //   ctrl2 = (ctrl2 & 0b11111000) | ((uint8_t)mode & 0b111);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL2);
//   // }

//   // /// Sets the motor direction (DIR).
//   // ///
//   // /// Allowed values are 0 or 1.
//   // ///
//   // /// You must first call enableSPIDirection() to allow the direction to be
//   // /// controlled through SPI.  Once you have done so, you can use this command
//   // /// to control the direction of the stepper motor and leave the DIR pin
//   // /// disconnected.
//   // void setDirection(bool value)
//   // {
//   //   if (value)
//   //   {
//   //     ctrl3 |= (1 << 7);
//   //   }
//   //   else
//   //   {
//   //     ctrl3 &= ~(1 << 7);
//   //   }
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL3);
//   // }

//   // /// Returns the cached value of the motor direction (DIR).
//   // ///
//   // /// This does not perform any SPI communication with the driver.
//   // bool getDirection()
//   // {
//   //   return (ctrl3 >> 7) & 1;
//   // }

//   // /// Advances the indexer by one step (STEP = 1).
//   // ///
//   // /// You must first call enableSPIStep() to allow stepping to be controlled
//   // /// through SPI.  Once you have done so, you can use this command to step the
//   // /// motor and leave the STEP pin disconnected.
//   // ///
//   // /// The driver automatically clears the STEP bit after it is written.
//   // void step(SPIClass *spi)
//   // {
//   //   driver.writeReg(spi, DRV8462RegAddr::SPI_CTRL3, ctrl3 | (1 << 6));
//   // }

//   // /// Enables direction control through SPI (SPI_DIR = 1), allowing
//   // /// setDirection() to override the DIR pin.
//   // void enableSPIDirection()
//   // {
//   //   ctrl3 |= (1 << 5);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL3);
//   // }

//   // /// Disables direction control through SPI (SPI_DIR = 0), making the DIR pin
//   // /// control direction instead.
//   // void disableSPIDirection()
//   // {
//   //   ctrl3 &= ~(1 << 5);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL3);
//   // }

//   // /// Enables stepping through SPI (SPI_STEP = 1), allowing step() to override
//   // /// the STEP pin.
//   // void enableSPIStep()
//   // {
//   //   ctrl3 |= (1 << 4);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL3);
//   // }

//   // /// Disables stepping through SPI (SPI_STEP = 0), making the STEP pin control
//   // /// stepping instead.
//   // void disableSPIStep()
//   // {
//   //   ctrl3 &= ~(1 << 4);
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL3);
//   // }

//   /// Sets the driver's stepping mode (MICROSTEP_MODE).
//   ///
//   /// This affects many things about the performance of the motor, including how
//   /// much the output moves for each step taken and how much current flows
//   /// through the coils in each stepping position.
//   ///
//   /// If an invalid stepping mode is passed to this function, then it selects
//   /// 1/16 micro-step, which is the driver's default.
//   ///
//   /// Example usage:
//   /// ~~~{.cpp}
//   /// sd.setStepMode(DRV8462StepMode::MicroStep32);
//   /// ~~~
//   // void setStepMode(DRV8462StepMode mode)
//   // {
//   //   if (mode > DRV8462StepMode::MicroStep256)
//   //   {
//   //     // Invalid mode; pick 1/16 micro-step by default.
//   //     mode = DRV8462StepMode::MicroStep16;
//   //   }

//   //   ctrl3 = (ctrl3 & 0b11110000) | (uint8_t)mode;
//   //   writeCachedReg(DRV8462RegAddr::SPI_CTRL3);
//   // }

//   // /// Sets the driver's stepping mode (MICROSTEP_MODE).
//   // ///
//   // /// This version of the function allows you to express the requested
//   // /// microstepping ratio as a number directly.
//   // ///
//   // /// Example usage:
//   // /// ~~~{.cpp}
//   // /// sd.setStepMode(32);
//   // /// ~~~
//   // void setStepMode(uint16_t mode)
//   // {
//   //   DRV8462StepMode sm;

//   //   switch (mode)
//   //   {
//   //     case 1:   sm = DRV8462StepMode::MicroStep1;   break;
//   //     case 2:   sm = DRV8462StepMode::MicroStep2;   break;
//   //     case 4:   sm = DRV8462StepMode::MicroStep4;   break;
//   //     case 8:   sm = DRV8462StepMode::MicroStep8;   break;
//   //     case 16:  sm = DRV8462StepMode::MicroStep16;  break;
//   //     case 32:  sm = DRV8462StepMode::MicroStep32;  break;
//   //     case 64:  sm = DRV8462StepMode::MicroStep64;  break;
//   //     case 128: sm = DRV8462StepMode::MicroStep128; break;
//   //     case 256: sm = DRV8462StepMode::MicroStep256; break;

//   //     // Invalid mode; pick 1/16 micro-step by default.
//   //     default:  sm = DRV8462StepMode::MicroStep16;
//   //   }

//   //   setStepMode(sm);
//   // }

//   /// Reads the FAULT status register of the driver.
//   ///
//   /// The return value is an 8-bit unsigned integer that has one bit for each
//   /// FAULT condition.  You can simply compare the return value to 0 to see if
//   /// any of the bits are set, or you can use the logical AND operator (`&`) and
//   /// the #DRV8462FaultBit enum to check individual bits.
//   ///
//   /// Example usage:
//   /// ~~~{.cpp}
//   /// if (sd.readFault() & (1 << (uint8_t)DRV8462FaultBit::UVLO))
//   /// {
//   ///   // Supply undervoltage lockout is active.
//   /// }
//   /// ~~~
//   // uint8_t readFault()
//   // {
//   //   return driver.readReg(DRV8462RegAddr::SPI_FAULT);
//   // }

//   /// Reads the DIAG1 status register of the driver.
//   ///
//   /// The return value is an 8-bit unsigned integer that has one bit for each
//   /// DIAG1 condition.  You can simply compare the return value to 0 to see if
//   /// any of the bits are set, or you can use the logical AND operator (`&`) and
//   /// the #DRV8462Diag1Bit enum to check individual bits.
//   // uint8_t readDiag1()
//   // {
//   //   return driver.readReg(DRV8462RegAddr::SPI_DIAG1);
//   // }

//   /// Reads the DIAG2 status register of the driver.
//   ///
//   /// The return value is an 8-bit unsigned integer that has one bit for each
//   /// DIAG2 condition.  You can simply compare the return value to 0 to see if
//   /// any of the bits are set, or you can use the logical AND operator (`&`) and
//   /// the #DRV8462Diag2Bit enum to check individual bits.
//   // uint8_t readDiag2()
//   // {
//   //   return driver.readReg(DRV8462RegAddr::SPI_DIAG2);
//   // }

//   /// Reads the DIAG3 status register of the driver.
//   ///
//   /// The return value is an 8-bit unsigned integer that has one bit for each
//   /// DIAG3 condition.  You can simply compare the return value to 0 to see if
//   /// any of the bits are set, or you can use the logical AND operator (`&`) and
//   /// the #DRV8462Diag3Bit enum to check individual bits.
//   // uint8_t readDiag3()
//   // {
//   //   return driver.readReg(DRV8462RegAddr::SPI_DIAG3);
//   // }

//   /// Clears any fault conditions that are currently latched in the driver
//   /// (CLR_FLT = 1).
//   ///
//   /// WARNING: Calling this function clears latched faults, which might allow
//   /// the motor driver outputs to reactivate.  If you do this repeatedly without
//   /// fixing an abnormal condition (like a short circuit), you might damage the
//   /// driver.
//   ///
//   /// The driver automatically clears the CLR_FLT bit after it is written.
//   // void clearFaults()
//   // {
//   //   driver.writeReg(spi, DRV8462RegAddr::SPI_CTRL4, ctrl4 | (1 << 7));
//   // }

//   /// Gets the cached value of a register. If the given register address is not
//   /// valid, this function returns 0.
//   // uint8_t getCachedReg(DRV8462RegAddr address)
//   // {
//   //   uint8_t * cachedReg = cachedRegPtr(address);
//   //   if (!cachedReg) { return 0; }
//   //   return *cachedReg;
//   // }

//   /// Writes the specified value to a register after updating the cached value
//   /// to match.
//   ///
//   /// Using this function keeps this object's cached settings consistent with
//   /// the settings being written to the driver, so if you are using
//   /// verifySettings(), applySettings(), and/or any of the other functions for
//   /// specific settings that this library provides, you should use this function
//   /// for direct register accesses instead of calling DRV8462SPI::writeReg()
//   /// directly.
//   // void setReg(DRV8462RegAddr address, uint8_t value)
//   // {
//   //   uint8_t * cachedReg = cachedRegPtr(address);
//   //   if (!cachedReg) { return; }
//   //   *cachedReg = value;
//   //   driver.writeReg(address, value);
//   // }

//   uint8_t testSetReg(SPIClass *spi, DRV8462RegAddr address, uint8_t value)
//   {
//     volatile uint16_t reg_value = 0;

//     reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         // Adding register address value
//     reg_value |= ((value << SPI_DATA_POS) & SPI_DATA_MASK);                 // Adding data value

//     driver.selectChip(spi);
//     // lastStatus = transfer((address & 0b11111) << 1);
//     uint8_t temp1 = driver.transfer(spi, (uint8_t)((reg_value>>8) & 0xFF));                // Transmit first Byte,MSB-Byte
//     // uint8_t oldData = transfer(value);
//     uint8_t oldData = driver.transfer(spi, (uint8_t)(reg_value & 0xFF));                // Transmit Second Byte, LSB-Byte
//     // The CS line must go low after writing for the value to actually take effect.
//     driver.deselectChip(spi);
//     return oldData;
//   }

// protected:

//   uint8_t ctrl1, ctrl2, ctrl3, ctrl4, ctrl5, ctrl6, ctrl7;

//   /// Returns a pointer to the variable containing the cached value for the
//   /// given register.
//   uint8_t * cachedRegPtr(DRV8462RegAddr address)
//   {
//     switch (address)
//     {
//       case DRV8462RegAddr::SPI_CTRL1: return &ctrl1;
//       case DRV8462RegAddr::SPI_CTRL2: return &ctrl2;
//       case DRV8462RegAddr::SPI_CTRL3: return &ctrl3;
//       case DRV8462RegAddr::SPI_CTRL4: return &ctrl4;
//       case DRV8462RegAddr::SPI_CTRL5: return &ctrl5;
//       case DRV8462RegAddr::SPI_CTRL6: return &ctrl6;
//       case DRV8462RegAddr::SPI_CTRL7: return &ctrl7;
//       default: return nullptr;
//     }
//   }

//   // /// Writes the cached value of the given register to the device.
//   // void writeCachedReg(DRV8462RegAddr address)
//   // {
//   //   uint8_t * cachedReg = cachedRegPtr(address);
//   //   if (!cachedReg) { return; }
//   //   // driver.writeReg(address, *cachedReg); // TEMP - comment back in
//   // }

// public:
//   /// This object handles all the communication with the DRV8711.  Generally,
//   /// you should not need to use it in your code for basic usage of a
//   /// High-Power Stepper Motor Driver, but you might want to use it to access
//   /// more advanced settings that the HighPowerStepperDriver class does not
//   /// provide functions for.
//   DRV8462SPI driver;
// };


#endif
