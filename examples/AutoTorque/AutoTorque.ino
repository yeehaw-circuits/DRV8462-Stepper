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

// #define DRV_ENABLE  2
// #define DRV_nSLEEP  4
// #define DRV_nFAULT  15
#define DRV_STEP    16
// #define DRV_DIR     17

// Initialize the DRV8462 instance
DRV8462 motorDriver = DRV8462();
// DRV8462 motorDriver = DRV8462(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS); // Custom pinout supported

// Auto Torque Setup
uint16_t CONST1 = 61;             // ATQ_LRN_CONST1
uint16_t CONST2 = 290;            // ATQ_LRN_CONST2
uint8_t ATQCountUpperLimit = 15;  // ATQ_UL
uint8_t ATQCountLowerLimit = 5;   // ATQ_LL
uint8_t MaxCurrentLimit = 180;    // ATQ_TRQ_MAX 
uint8_t MinCurrentLimit = 40;     // ATQ_TRQ_MIN 
uint8_t KP = 1;                   // KP
uint8_t KD = 0;                   // KD

// Values for 17HS24 2.1A - https://www.omc-stepperonline.com/nema-17-bipolar-1-8deg-65ncm-92oz-in-2-1a-3-36v-42x42x60mm-4-wires-17hs24-2104s
// uint16_t CONST1 = 61;             // ATQ_LRN_CONST1
// uint16_t CONST2 = 290;            // ATQ_LRN_CONST2
// uint8_t ATQCountUpperLimit = 15;  // ATQ_UL
// uint8_t ATQCountLowerLimit = 5;   // ATQ_LL
// uint8_t MaxCurrentLimit = 180;    // ATQ_TRQ_MAX 
// uint8_t MinCurrentLimit = 40;     // ATQ_TRQ_MIN 
// uint8_t KP = 1;                   // KP
// uint8_t KD = 0;                   // KD

// Values for 17HS16 2.0A - https://www.omc-stepperonline.com/nema-17-bipolar-45ncm-64oz-in-2a-42x42x40mm-4-wires-w-1m-cable-connector-17hs16-2004s1
// with KP=2, KD=1
// uint16_t CONST1 = 47;
// uint16_t CONST2 = 211;
// uint8_t ATQCountUpperLimit = 20;   // ATQ_UL
// uint8_t ATQCountLowerLimit = 5;   // ATQ_LL
// uint8_t MaxCurrentLimit = 180;    // ATQ_TRQ_MAX 
// uint8_t MinCurrentLimit = 30;     // ATQ_TRQ_MIN 

uint8_t ATQCountUpperLimitAsPercent = (int)ATQCountUpperLimit / 255;
uint8_t ATQCountLowerLimitAsPercent = (int)ATQCountLowerLimit / 255;
uint8_t MaxCurrentLimitAsPercent = (int)MaxCurrentLimit / 255;
uint8_t MinCurrentLimitAsPercent = (int)MinCurrentLimit / 255;

int x;
String str;

void setup() {
  Serial.begin(115200); 
  Serial.setTimeout(500);

  motorDriver.FullInitialize();
  // motorDriver.setRunCurrentPercent(50); 
  motorDriver.setTorqueDAC(127); 
  motorDriver.disableAutoTorqueMode();
  motorDriver.setMicrostepMode(MicroStep16); // Set the microstepping mode using names
  motorDriver.setDirectionForward();
  motorDriver.enableOutputs(); // Set the ENABLE pin HIGH
  

  // Easy PWM generation using LEDc on ESP32
  ledcSetup(0, PWM_Frequency, 8);  // Channel 0, default to 5000kHz, 8-bit resolution (0-255)
  ledcAttachPin(DRV_STEP, 0);
  ledcWrite(0, 127); // 50% duty cycle

  delay(2000);

  // Auto Torque Setup following Figure 7-25. Auto-torque Learning Flowchart
  motorDriver.enableAutoTorqueMode();
  motorDriver.ATQ_SetInitialLearnCurrent(4);
  motorDriver.ATQ_SetCurrentStepsForLearn(128);
  motorDriver.ATQ_SetCurrentCyclesForLearn(24);
  motorDriver.ATQ_SetKP(2);
  motorDriver.ATQ_SetKD(1);
  motorDriver.ATQ_SetATQCountUpperLimit(ATQCountUpperLimit);  // (upper dashed blue line)
  motorDriver.ATQ_SetATQCountLowerLimit(ATQCountLowerLimit);  // (lower dashed blue line)
  motorDriver.ATQ_SetMaxCurrentLimit(MaxCurrentLimit);        // (upper dashed orange line)
  motorDriver.ATQ_SetMinCurrentLimit(MinCurrentLimit);        // (lower dashed orange line)

  
  // Start auto torque learning
  motorDriver.ATQ_StartLearning();
  delay(1000);

  // Print and apply the learned values
  CONST1 = motorDriver.ATQ_ReadATQ_LRN_CONST1();
  CONST2 = motorDriver.ATQ_ReadATQ_LRN_CONST2();
  Serial.print("ATQ_LRN_CONST1:");  Serial.println(CONST1);
  Serial.print("ATQ_LRN_CONST2:");  Serial.println(CONST2);
  motorDriver.disableAutoTorqueMode();
  delay(500);

  // motorDriver.ATQ_SetATQ_LRN_CONST1and2(CONST1, CONST2);
  // motorDriver.ATQ_SetATQCountUpperLimit(ATQCountUpperLimit);  // (upper dashed blue line)
  // motorDriver.ATQ_SetATQCountLowerLimit(ATQCountLowerLimit);  // (lower dashed blue line)
  // motorDriver.ATQ_SetMaxCurrentLimit(MaxCurrentLimit);        // (upper dashed orange line)
  // motorDriver.ATQ_SetMinCurrentLimit(MinCurrentLimit);        // (lower dashed orange line)
  // motorDriver.ATQ_SetKP(1);
  // motorDriver.ATQ_SetKD(0);

  motorDriver.enableAutoTorqueMode();
  delay(25);

  motorDriver.enableOutputs();
  ledcWrite(0, 127); // start the STEP output



}

void loop() {

  // Best used with the Arduino Serial Plotter

  // Read and print the changing values from the device
  Serial.print("ATQ_DAC:");         Serial.print(motorDriver.ATQ_ReadTRQ_DAC());              Serial.print(",");
  Serial.print("ATQ_CNT:");         Serial.print(motorDriver.ATQ_ReadATQ_CNT());              Serial.print(",");

  Serial.print("ATQ_UL:");          Serial.print(motorDriver.ATQ_ReadATQCountUpperLimit());   Serial.print(",");
  Serial.print("ATQ_LL:");          Serial.print(motorDriver.ATQ_ReadATQCountLowerLimit());   Serial.print(",");
  Serial.print("ATQ_TRQ_MAX:");     Serial.print(motorDriver.ATQ_ReadMaxCurrentLimit());      Serial.print(",");
  Serial.print("ATQ_TRQ_MIN:");     Serial.print(motorDriver.ATQ_ReadMinCurrentLimit());      Serial.print(",");

  Serial.print("ATQ_LRN_CONST1:");  Serial.print(motorDriver.ATQ_ReadATQ_LRN_CONST1());       Serial.print(",");
  Serial.print("ATQ_LRN_CONST2:");  Serial.print(motorDriver.ATQ_ReadATQ_LRN_CONST2());       Serial.print(",");

  Serial.println();

  if(Serial.available() > 0)
    {
      String commandName = "";
      int newValueInt = -1;
      
      // Peek to see if the command was "startLearn" or just s
      int commandNameTemp = Serial.peek();
      Serial.println(commandNameTemp);
      if (commandNameTemp == 115 || commandNameTemp == 83)
      {
          String trash1 = Serial.readStringUntil('\n');
          motorDriver.ATQ_StartLearning();
          delay(1000);
          CONST1 = motorDriver.ATQ_ReadATQ_LRN_CONST1();
          CONST2 = motorDriver.ATQ_ReadATQ_LRN_CONST2();
          motorDriver.disableAutoTorqueMode();
          delay(50);
          motorDriver.enableAutoTorqueMode();
          delay(20);
      }
      else
      {
        commandName = Serial.readStringUntil(':');
        newValueInt = Serial.parseInt();
        String trash = Serial.readStringUntil('\n'); // Read to the end of the Serial input 

        uint8_t newValue = constrain(newValueInt,0,255);

        if(commandName.equalsIgnoreCase("min"))
        {
          MinCurrentLimit = newValue;
          motorDriver.ATQ_SetMinCurrentLimit(newValue); 
        }
        else if(commandName.equalsIgnoreCase("max"))
        {
          MaxCurrentLimit = newValue;
          motorDriver.ATQ_SetMaxCurrentLimit(newValue); 
        }
        else if(commandName.equalsIgnoreCase("UL") || commandName.equalsIgnoreCase("upper"))
        {
          ATQCountUpperLimit = newValue;
          motorDriver.ATQ_SetATQCountUpperLimit(newValue); 
        }
        else if(commandName.equalsIgnoreCase("LL") || commandName.equalsIgnoreCase("lower"))
        {
          ATQCountLowerLimit = newValue;
          motorDriver.ATQ_SetATQCountLowerLimit(newValue); 
        }
        else if(commandName.equalsIgnoreCase("C1") || commandName.equalsIgnoreCase("CONST1"))
        {
          // CONST1 and CONST2 are 10-bit values, so max of 1023 instead of 255
          uint16_t newValueForConst = constrain(newValueInt,0,1023);
          CONST1 = newValueForConst;
          CONST2 = motorDriver.ATQ_ReadATQ_LRN_CONST2();
          motorDriver.ATQ_SetATQ_LRN_CONST1and2(CONST1, CONST2);
        }
        else if(commandName.equalsIgnoreCase("C2") || commandName.equalsIgnoreCase("CONST2"))
        {
          // CONST1 and CONST2 are 10-bit values, so max of 1023 instead of 255
          uint16_t newValueForConst = constrain(newValueInt,0,1023);
          CONST2 = newValueForConst;
          CONST1 = motorDriver.ATQ_ReadATQ_LRN_CONST1();
          motorDriver.ATQ_SetATQ_LRN_CONST1and2(CONST1, CONST2);
        }
        else if(commandName.equalsIgnoreCase("KP") || commandName.equalsIgnoreCase("P"))
        {
          motorDriver.ATQ_SetKP(newValue);
        }
        else if(commandName.equalsIgnoreCase("KD") || commandName.equalsIgnoreCase("D"))
        {
          motorDriver.ATQ_SetKD(newValue);
        }
        else if(commandName.equalsIgnoreCase("KD") || commandName.equalsIgnoreCase("D"))
        {
          motorDriver.ATQ_SetKD(newValue);
        }
      }
    }


  delay(100);

}


