/*
 *  variant.cpp for Macchina M2
 *
 *   Author: Tony Doust
 *
 * Short description:
 *   Macchina M2 Arduino_DUE Variant PIN Numbering to PIN Definition Names
 */

/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

/*
 *
 *  DUE Board pin |  PORT  | Label
 * ---------------+--------+-------
 *  0    (Serial) |  PA8   | "XBEE_RX/RX0"
 *  1             |  PA9   | "XBEE_TX/TX0"
 *  2             |  PB25  | "XBEE_RTS"
 *  3             |  PB26  | "XBEE_CTS"
 *  4             |  PC11  | "XBEE_RST"
 *  5             |  PC13  | "XBEE_STAT"
 *  6             |  PC14  | "XBEE_VREF"
 *  7             |  PB3   | "XBEE_PWM"
 *  8             |  PC12  | "XBEE_MULT1"
 *  9             |  PC15  | "XBEE_MULT2"
 *  10            |  PC17  | "XBEE_MULT3"
 *  11            |  PD6   | "XBEE_MULT4"
 *  12            |  PD9   | "XBEE_MULT5"
 *  13            |  PA7   | "XBEE_MULT6"
 *  14            |  PD10  | "DS2"              // Red LED
 *  15            |  PA5   | "DS3"              // Yellow LED
 *  16            |  PD2   | "DS4"              // Yellow LED
 *  17            |  PA15  | "DS5"              // Yellow LED
 *  18            |  PA14  | "DS6"              // Green LED
 *  19            |  PC25  | "DS7_BLUE/RGB_BLUE"
 *  20            |  PD7   | "DS7_RED/RGB_RED"
 *  21            |  PD8   | "DS7_GREEN/RGB_GREEN"
 *  22            |  PC27  | "Button1"          // M2_Button_1
 *  23            |  PB6A  | "Button2"          // M2_Button_2
 *  24      PWMH0 |  PC3   | "GPIO1"
 *  25      PWMH1 |  PC5   | "GPIO2"
 *  26      PWMH2 |  PC7   | "GPIO3"
 *  27      PWMH3 |  PC9   | "GPIO4"
 *  28      PWMH4 |  PC20  | "GPIO5"
 *  29      PWMH5 |  PC19  | "GPIO6"
 *  30      PWML0 |  PC2   | "GPIO1_B"          // legacy M2 Beta
 *  31      PWML1 |  PC4   | "GPIO2_B"          // legacy M2 Beta
 *  32      PWML2 |  PC6   | "GPIO3_B"          // legacy M2 Beta
 *  33      PWML3 |  PC8   | "GPIO4_B"          // legacy M2 Beta
 *  34      PWML4 |  PC21  | "GPIO5_B"          // legacy M2 Beta
 *  35      PWML5 |  PC22  | "GPIO6_B"          // legacy M2 Beta
 *  36            |  PC30  | "SD_SW"
 *  37            |  PA19  | "MCCK"
 *  38            |  PA20  | "MCCDA"
 *  39            |  PA21  | "MCDA0"
 *  40            |  PA22  | "MCDA1"
 *  41            |  PA23  | "MCDA2"
 *  42            |  PA24  | "MCDA3"
 *  43      MISO  |  PA25  | "SPI0_MISO"        // 26 Pin Connector
 *  44      MOSI  |  PA26  | "SPI0_MOSI"        // 26 Pin Connector
 *  45      SCLK  |  PA27  | "SPI0_CLK"         // 26 Pin Connector
 *  46      NPCS1 |  PA29  | "SPI0_CS1"
 *  47            |  PA28  | "SPI0_CS0"
 *  48            |  PC10  | "PS_BUCK/BUCK_DIS"
 *  49            |  PB5   | "PS_J1850_9141"
 *  50            |  PB8   | "J1850_PWM_VPW"
 *  51            |  PC28  | "J1850_PWM_RX"
 *  52            |  PC26  | "J1850_VPW_RX"
 *  53            |  PC18  | "J1850P_TX"
 *  54            |  PC23  | "J1850N_TX"
 *  55  (Serial1) |  PA11  | "LIN_KTX"
 *  56            |  PA10  | "LIN_KRX"
 *  57            |  PB4   | "LIN_KSLP"
 *  58  (Serial2) |  PA13  | "LIN_LTX"
 *  59            |  PA12  | "LIN_LRX"
 *  60            |  PB7   | "LIN_LSLP"
 *  61            |  PB1   | "SWC_RX0"
 *  62            |  PB2   | "SWC_RX1"
 *  63            |  PB23  | "SPI0_CS3"
 *  64            |  PB27  | "SWC_M0"
 *  65            |  PB0   | "SWC_M1"
 *  66            |  PB22  | "SWC_CLK"
 *  67            |  PC16  | "SWC_INT"
 *  68            |  PC29  | "SWC_SOF"
 *  69            |  PA1   | "CANRX0"
 *  70            |  PA0   | "CANTX0"
 *  71            |  PD3   | "CAN0_CS/HS_CS"
 *  72            |  PB15  | "CANRX1"
 *  73            |  PB14  | "CANTX1"
 *  74            |  PD0   | "CAN1_CS/MS_CS"
 *  75            |  PC24  | "I_SENSE_EN"
 *  76            |  PD1   | "I_SENSE_INT"
 *  77  (Serial3) |  PD4   | "TXD3/UART3"       // 26 Pin Connector
 *  78            |  PD5   | "RXD3/UART3"       // 26 Pin Connector
 *  79            |  PA17  | "SDA0"             // 26 Pin Connector
 *  80            |  PA18  | "SCL0"             // 26 Pin Connector
 *  81            |  PB12  | "SDA1"
 *  82            |  PB13  | "SCL1"
 *  83            |  PB21  | "SPI_CS2"
 *  84            |  PB20  | "USART2TX"     // Test Point 5
 *  85            |  PC1   | "unconnected!" // ** unconnected processor pin ** //
 *       Analogue Pins
 *      --------------
 *  86            |  PB19  | "ANALOG_1/A0"     // AD10
 *  87            |  PB18  | "ANALOG_2/A1"     // AD9
 *  88            |  PA2   | "ANALOG_3/A2"     // AD7
 *  89            |  PA4   | "ANALOG_4/A3"     // AD5
 *  90            |  PA3   | "ANALOG_5/A4"     // AD6
 *  91            |  PA16  | "ANALOG_6/A5"     // AD0
 *  92            |  PA6   | "V_SENSE/A6"      // AD3
 *  93            |  PB17  | "I_SENSE/A7"      // AD8
 *  94            |  PD5   | "CPU_TEMP/A8"     // CPU Temperature
 *  95            |  PB16  | "I_SENSE_DAC"     // DAC1
 *  96  Pins Masks|  PA17A | "TWI0"
 *  97            |  PB12A | "TWI1"
 *  98            |  PA8A  | "UART"
 *  99    to      |  PA11A | "USART0"
 *  100           |  PA13A | "USART1"
 *  101 Pins Masks|  PD4B  | "USART3"
 *  102           |  PB11  | "USB" (UOTGID)
 *  103           |  PB21  | "USART2RX"         // 26 Pin Connector
 *  104           |  PB15  | "CAN1RX"
 *  105           |  PB14  | "CAN1TX"
 *  106 CAN Masks |  PA1A  | "PINS_CAN0"        // Can0 Pin mask for TX & RX
 *  107 CAN Masks |  PB15A | "PINS_CAN1"        // Can1 Pin mask for TX & RX
 *
 *  USB pin       |  PORT
 * ---------------+--------
 *  ID            |  PB11
 *  VBOF          |  PB10
 *
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
    // **************************************************************************** //
    //                          M2 PIN Definitions                                  //
    // **************************************************************************** //
  // 0 .. 85 - Digital pins
  // ----------------------
  // 0 .. 1 - UART (Serial)
// pPort,   ulPin,      ulPeripheralId, ulPinType, ulPinConfiguration,ulPinAttribute,   ulAnalogChannel,ulADCChannelNumber,ulPWMChannel,ulTCChannel
  { PIOA, PIO_PA8A_URXD,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_RX
  { PIOA, PIO_PA9A_UTXD,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_TX

  // 2 .. 13 XBEE Interface
  { PIOB, PIO_PB25B_TIOA0,  ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),  NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHA0     }, // XBEE_RTS
  { PIOB, PIO_PB26,         ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_CTS
  { PIOC, PIO_PC11,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_RST
  { PIOC, PIO_PC13,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_STAT
  { PIOC, PIO_PC14,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_VREF
  { PIOB, PIO_PB3A_ETX1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_PWM
  { PIOC, PIO_PC12,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT1
  { PIOC, PIO_PC15,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT2
  { PIOC, PIO_PC17,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT3
  { PIOD, PIO_PD6,          ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT4
  { PIOD, PIO_PD9,          ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT5
  { PIOA, PIO_PA7,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // XBEE_MULT6
  // 14 .. 18 LEDS
  { PIOD, PIO_PD10,         ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS2 // RED LED
  { PIOA, PIO_PA5,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS3 // YELLOW LED
  { PIOD, PIO_PD2,          ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS4 // YELLOW LED
  { PIOA, PIO_PA15,         ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS5 // YELLOW LED
  { PIOA, PIO_PA14,         ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DS6 // GREEN LED
  // 19 .. 21 RGB LEDS
  { PIOC, PIO_PC25B_TIOA6,  ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),   NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA6     }, // DS7_BLUE/RGB_BLUE
  { PIOD, PIO_PD7B_TIOA8,   ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),   NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA8     }, // DS7_RED/RGB_RED
  { PIOD, PIO_PD8B_TIOB8,   ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),   NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB8     }, // DS7_GREEN/RGB_GREEN
  // 22 .. 23 M2_Button_1 & M2_Button_2
  { PIOC, PIO_PC27,         ID_PIOC, PIO_PERIPH_A, (PIO_DEFAULT|PIO_DEBOUNCE), PIN_ATTR_DIGITAL,    NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // Button1
  { PIOB, PIO_PB6A_ERX1,    ID_PIOB, PIO_PERIPH_A, (PIO_DEFAULT|PIO_DEBOUNCE), PIN_ATTR_DIGITAL,    NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // Button2
  // 24 .. 29 GPIO1 - GPIO6
  { PIOC, PIO_PC3,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH0,     NOT_ON_TIMER }, // GPIO1
  { PIOC, PIO_PC5,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH1,     NOT_ON_TIMER }, // GPIO2
  { PIOC, PIO_PC7,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH2,     NOT_ON_TIMER }, // GPIO3
  { PIOC, PIO_PC9,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH3,     NOT_ON_TIMER }, // GPIO4
  { PIOC, PIO_PC20,         ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH4,     NOT_ON_TIMER }, // GPIO5
  { PIOC, PIO_PC19,         ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH5,     NOT_ON_TIMER }, // GPIO6
  // 30 .. 35 GPIO1_B - GPIO6_B Beta Legacy pins
  { PIOC, PIO_PC2,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH0,     NOT_ON_TIMER }, // GPIO1_B
  { PIOC, PIO_PC4,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH1,     NOT_ON_TIMER }, // GPIO2_B
  { PIOC, PIO_PC6,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH2,     NOT_ON_TIMER }, // GPIO3_B
  { PIOC, PIO_PC8,          ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH3,     NOT_ON_TIMER }, // GPIO4_B
  { PIOC, PIO_PC21,         ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH4,     NOT_ON_TIMER }, // GPIO5_B
  { PIOC, PIO_PC22,         ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH5,     NOT_ON_TIMER }, // GPIO6_B
  // 36 SD Card Inserted
  { PIOC, PIO_PC30,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SD_SW
  //37 .. 42 SD Card Control & Data
  { PIOA, PIO_PA19A_MCCK,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCCK
  { PIOA, PIO_PA20A_MCCDA,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCCDA
  { PIOA, PIO_PA21A_MCDA0,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA0
  { PIOA, PIO_PA22A_MCDA1,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA1
  { PIOA, PIO_PA23A_MCDA2,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA2
  { PIOA, PIO_PA24A_MCDA3,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MCDA3
  // 43 .. 45 SPI0
  { PIOA, PIO_PA25A_SPI0_MISO,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_MISO
  { PIOA, PIO_PA26A_SPI0_MOSI,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_MOSI
  { PIOA, PIO_PA27A_SPI0_SPCK,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_CLK
  // 46 - SPI0_CS1
  { PIOA, PIO_PA29A_SPI0_NPCS1,ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_CS1
  // 47 - SPI0_CS0
  { PIOA, PIO_PA28A_SPI0_NPCS0,ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_CS/SPI0_CS0
  // 48 5Volt Power Supply Buck Boost
  { PIOC, PIO_PC10,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PS_BUCK/BUCK_DIS
  // 49 J1850_9141_ON
  { PIOB, PIO_PB5A_ERX0,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER  }, // PS_J1850_9141
  // 50 J1850_PWM_VPW
  { PIOB, PIO_PB8A_EMDC,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER  }, // J1850_PWM_VPW
  { PIOC, PIO_PC28B_TIOA7,  ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),   NO_ADC, NO_ADC, NOT_ON_PWM, TC2_CHA7      }, // J1850_PWM_RX
  { PIOC, PIO_PC26B_TIOB6,  ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),   NO_ADC, NO_ADC, NOT_ON_PWM, TC2_CHB6      }, // J1850_VPW_RX
  { PIOC, PIO_PC18,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER  }, // J1850P_TX
  { PIOC, PIO_PC23B_PWML6,  ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_PWM),     NO_ADC, NO_ADC, PWM_CH6,    NOT_ON_TIMER  }, // J1850N_TX
  // 55 - 56 USART0 (Serial1)
  { PIOA, PIO_PA11A_TXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER  }, // LIN_KTX/9141_KTX
  { PIOA, PIO_PA10A_RXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER  }, // LIN_KRX/9141_KRX
  // 57 LIN_KSLP
  { PIOB, PIO_PB4A_ECRSDV,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER  }, // LIN_KSLP/9141_KSLP
  // 58 .. 59 USART1 (Serial2)
  { PIOA, PIO_PA13A_TXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LIN_LTX/9141_LTX
  { PIOA, PIO_PA12A_RXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LIN_LRX/9141_LRX
  // 60 LIN_LSLP
  { PIOB, PIO_PB7A_ERXER,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LIN_LSLP/9141_LSLP
  // 61 .. 68 SWC (Single Wire Can)
  { PIOB, PIO_PB1A_ETXEN,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SWC_RX0
  { PIOB, PIO_PB2A_ETX0,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SWC_RX1
  { PIOB, PIO_PB23B_SPI0_NPCS3,ID_PIOB,PIO_PERIPH_B,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPI0_CS3
  { PIOB, PIO_PB27B_TIOB0,  ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),   NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHB0     }, // SWC_M0
  { PIOB, PIO_PB0A_ETXCK,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SCW_M1
  { PIOB, PIO_PB22,         ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SWC CLK
  { PIOC, PIO_PC16,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                   NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SWC_INT
  { PIOC, PIO_PC29B_TIOB7,  ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_TIMER),   NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB7     }, // SWC_SOF
  //69 .. 74 CAN0 & CAN1
  { PIOA, PIO_PA1A_CANRX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  ADC14,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX0
  { PIOA, PIO_PA0A_CANTX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  ADC15,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANTX0
  { PIOD, PIO_PD3,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CAN0_CS/HS_CS
  { PIOB, PIO_PB15X1_DAC0,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC12,  DA0,    NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX1    // DAC0
  { PIOB, PIO_PB14,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANTX1
  { PIOD, PIO_PD0,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CAN1_CS/MS_CS
  //75 .. 76 I_SENSE I/O Power Supply
  { PIOC, PIO_PC24B_PWML7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // I_SENSE_EN
  { PIOD, PIO_PD1,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // I_SENSE_INT
  //77 .. 78 USART3 (Serial3)
  { PIOD, PIO_PD4B_TXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UART3/TXD3" // 26 Pin Connector
  { PIOD, PIO_PD5B_RXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UART3/RXD3" // 26 Pin Connector
  // 79 .. 80 - TWI0
  { PIOA, PIO_PA17A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SDA0
  { PIOA, PIO_PA18A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SCL0
  // 81 .. 82 - TWI1
  { PIOB, PIO_PB12A_TWD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SDA1
  { PIOB, PIO_PB13A_TWCK1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SCL1
  //83 SPI_CS2
  { PIOB, PIO_PB21,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SS2 // USART2RX
  { PIOB, PIO_PB20X1_AD13,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC11,  ADC13,  NOT_ON_PWM,  NOT_ON_TIMER }, // USART2TX // AD11 ** Check **
  { PIOC, PIO_PC1,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // unconnected!

  // 86 .. 93 - Analog pins
  // ----------------------
  { PIOB, PIO_PB19X1_AD12,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC0,   ADC12,  NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_1  // ADC12
  { PIOB, PIO_PB18X1_AD11,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC1,   ADC11,  NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_2  // AD9
  { PIOA, PIO_PA2X1_AD0,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC2,   ADC0,   NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_3  // AD7
  { PIOA, PIO_PA4X1_AD2,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC3,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_4  // AD5
  { PIOA, PIO_PA3X1_AD1,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC4,   ADC1,   NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_5  // AD6
  { PIOA, PIO_PA16X1_AD7,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC5,   ADC7,   NOT_ON_PWM,  NOT_ON_TIMER }, // ANALOG_6  // AD0
  { PIOA, PIO_PA6X1_AD3,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC6,   ADC3,   NOT_ON_PWM,  NOT_ON_TIMER }, // V_SENSE   // ADC3
  { PIOB, PIO_PB17X1_AD10,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC7,   ADC10,  NOT_ON_PWM,  NOT_ON_TIMER }, // I_SENSE   // AD8
  // 94 AD15 CPU Chip Temperature
  { PIOD, PIO_PD5A_A15,     ID_PIOD, PIO_NOT_A_PIN, PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC8,   ADC15, NOT_ON_PWM,  NOT_ON_TIMER   }, // CPU_TEMP/A15 // ADC15 CPU Temperature

  // 95 - CANRX1/DAC1
  { PIOB, PIO_PB16X1_DAC1,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC13,  DA1,    NOT_ON_PWM,  NOT_ON_TIMER	}, // I_SENSE_DAC // DAC1

  // 96 .. 101 - "All pins" masks
  //-----------------------------

  // 96 - TWI0 all pins
  { PIOA, PIO_PA17A_TWD0|PIO_PA18A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 97 - TWI1 all pins
  { PIOB, PIO_PB12A_TWD1|PIO_PB13A_TWCK1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 98 - UART (Serial) all pins XBEE_RX, XBEE_TX
  { PIOA, PIO_PA8A_URXD|PIO_PA9A_UTXD, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),    NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 99 - USART0 (Serial1) all pins LIN_KTX, LIN_RTX
  { PIOA, PIO_PA11A_TXD0|PIO_PA10A_RXD0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 100 - USART1 (Serial2) all pins LIN_LTX, LIN_LRX
  { PIOA, PIO_PA13A_TXD1|PIO_PA12A_RXD1, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 101 - USART3 (Serial3) all pins UART3 TXD3, RXD3 26 pin connector
  { PIOD, PIO_PD4B_TXD3|PIO_PD5B_RXD3, ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),    NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  // 102 - USB (UOTGID)
  { PIOB, PIO_PB11A_UOTGID|PIO_PB10A_UOTGVBOF, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,            NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // USB (UOTGID)

  // 103 - USART2RX
  { PIOB, PIO_PB21B_SPI0_NPCS2, ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                           NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // USART2RX

  // 104 .. 105 - CAN1RX/CAN1TX (same physical pin for 66/53)
  { PIOB, PIO_PB15A_CANRX1,     ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                           NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CAN1RX
  { PIOB, PIO_PB14A_CANTX1,     ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                           NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANT1X

  // 106 .. 107 - "All CAN pins" masks
  // 106 - CAN0 all pins
  { PIOA, PIO_PA1A_CANRX0|PIO_PA0A_CANTX0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),    NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },
  // 107 - CAN1 all pins
  { PIOB, PIO_PB15A_CANRX1|PIO_PB14A_CANTX1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT,(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },
    // END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;


uint8_t g_pinStatus[PINS_COUNT] = {0};

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
RingBuffer rx_buffer1;
RingBuffer tx_buffer1;

UARTClass Serial(UART, UART_IRQn, ID_UART, &rx_buffer1, &tx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }

void serialEventUSB() __attribute__((weak));
void serialEventUSB() { }

// IT handlers
void UART_Handler(void)
{
  Serial.IrqHandler();
}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;
RingBuffer rx_buffer4;
RingBuffer tx_buffer2;
RingBuffer tx_buffer3;
RingBuffer tx_buffer4;

USARTClass Serial1(USART0, USART0_IRQn, ID_USART0, &rx_buffer2, &tx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
USARTClass Serial2(USART1, USART1_IRQn, ID_USART1, &rx_buffer3, &tx_buffer3);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }
USARTClass Serial3(USART3, USART3_IRQn, ID_USART3, &rx_buffer4, &tx_buffer4);
void serialEvent3() __attribute__((weak));
void serialEvent3() { }

// IT handlers
void USART0_Handler(void)
{
  Serial1.IrqHandler();
}

void USART1_Handler(void)
{
  Serial2.IrqHandler();
}

void USART3_Handler(void)
{
  Serial3.IrqHandler();
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
  if (Serial3.available()) serialEvent3();
  if (SerialUSB.available()) serialEventUSB();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
  SystemInit();

  // Set Systick to 1ms interval, common to all SAM3 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
    while (true);
  }

  // Initialize C library
  __libc_init_array();

  // Disable pull-up on every pin
  for (unsigned i = 0; i < PINS_COUNT; i++)
      digitalWrite(i, LOW);

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  PIOC->PIO_OWER = 0xFFFFFFFF;
  PIOD->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port U(S)ART pins
  PIO_Configure(
    g_APinDescription[PINS_UART].pPort,
    g_APinDescription[PINS_UART].ulPinType,
    g_APinDescription[PINS_UART].ulPin,
    g_APinDescription[PINS_UART].ulPinConfiguration);
  digitalWrite(0, HIGH); // Enable pullup for RX0
  PIO_Configure(
    g_APinDescription[PINS_USART0].pPort,
    g_APinDescription[PINS_USART0].ulPinType,
    g_APinDescription[PINS_USART0].ulPin,
    g_APinDescription[PINS_USART0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART1].pPort,
    g_APinDescription[PINS_USART1].ulPinType,
    g_APinDescription[PINS_USART1].ulPin,
    g_APinDescription[PINS_USART1].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART3].pPort,
    g_APinDescription[PINS_USART3].ulPinType,
    g_APinDescription[PINS_USART3].ulPin,
    g_APinDescription[PINS_USART3].ulPinConfiguration);

  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pPort,
    g_APinDescription[PINS_USB].ulPinType,
    g_APinDescription[PINS_USB].ulPin,
    g_APinDescription[PINS_USB].ulPinConfiguration);

  // Initialize CAN pins
  PIO_Configure(
    g_APinDescription[PINS_CAN0].pPort,
    g_APinDescription[PINS_CAN0].ulPinType,
    g_APinDescription[PINS_CAN0].ulPin,
    g_APinDescription[PINS_CAN0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_CAN1].pPort,
    g_APinDescription[PINS_CAN1].ulPinType,
    g_APinDescription[PINS_CAN1].ulPin,
    g_APinDescription[PINS_CAN1].ulPinConfiguration);

  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);

  adc_disable_all_channel(ADC);
  adc_enable_channel(ADC, ADC_CHANNEL_12); // Analog1
  adc_enable_channel(ADC, ADC_CHANNEL_11); // Analog2
  adc_enable_channel(ADC, ADC_CHANNEL_0);  // Analog3
  adc_enable_channel(ADC, ADC_CHANNEL_2);  // Analog4
  adc_enable_channel(ADC, ADC_CHANNEL_1);  // Analog5
  adc_enable_channel(ADC, ADC_CHANNEL_7);  // Analog6
  adc_enable_channel(ADC, ADC_CHANNEL_3);  // V_SENSE
  adc_enable_channel(ADC, ADC_CHANNEL_10); // I_SENSE

  adc_enable_ts(ADC); // Enable A15 Chip Temeprature tson bit
  adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);

  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.


  // Initialize analogOutput module
  analogOutputInit();
}

#ifdef __cplusplus
}
#endif

