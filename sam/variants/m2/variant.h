/*
*  variant.h for Macchina M2
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

#ifndef _VARIANT_MACCHINA_M2_
#define _VARIANT_MACCHINA_M2_


/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC     12000000

/** Master clock frequency */
#define VARIANT_MCK         84000000

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (96u)
#define NUM_DIGITAL_PINS     (86u)
#define NUM_ANALOG_INPUTS    (9u)
#define analogInputToDigitalPin(p)  ((p < 9u) ? (p) + 86u : -1)

#define digitalPinToPort(P)        ( g_APinDescription[P].pPort )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->PIO_ODSR) )
#define portInputRegister(port)    ( &(port->PIO_PDSR) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * portModeRegister(..) should return a register to set pin mode
 * INPUT or OUTPUT by setting the corresponding bit to 0 or 1.
 * Unfortunately on SAM architecture the PIO_OSR register is
 * read-only and can be set only through the enable/disable registers
 * pair PIO_OER/PIO_ODR.
 */
// #define portModeRegister(port)   ( &(port->PIO_OSR) )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAM
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// Interrupts
#define digitalPinToInterrupt(p)  ((p) < NUM_DIGITAL_PINS ? (p) : -1)

// **************************************************************************** //
//                          M2 PIN Definitions                                  //
// **************************************************************************** //

// M2 LEDs
#define DS2             14  // (RED)
#define DS3             15  // (YELLOW)
#define DS4             16  // (YELLOW)
#define DS5             17  // (YELLOW)
#define DS6             18  // (GREEN)

#define DS7_BLUE        19  // RGB Blue LED
#define DS7_RED         20  // RGB Red LED
#define DS7_GREEN       21  // RGB Green LED

#define RGB_RED    DS7_RED      // RGB Red LED
#define RGB_GREEN  DS7_GREEN    // RGB Green LED
#define RGB_BLUE   DS7_BLUE     // RGB Blue LED


#define LED_BUILTIN     RGB_BLUE
/*
#define PIN_LED_13      DS2
#define PIN_LED_RXL     DS7_GREEN
#define PIN_LED_TXL     DS7_BLUE
#define PIN_LED         PIN_LED_13
#define PIN_LED2        PIN_LED_RXL
#define PIN_LED3        PIN_LED_TXL
*/

// M2 GPIO
#define GPIO1           24
#define GPIO2           25
#define GPIO3           26
#define GPIO4           27
#define GPIO5           28
#define GPIO6           29

#ifdef MACCHINA_M2_BETA  // M2 Beta legacy Hardware Sink Input Pins
    // M2 GPIO_B pins for Sinking INPUT Pins
    #define GPIO1_B     30
    #define GPIO2_B     31
    #define GPIO3_B     32
    #define GPIO4_B     33
    #define GPIO5_B     34
    #define GPIO6_B     35
#endif


// M2 User Buttons
#define Button1         22
#define Button2         23


// M2 Analogue GPIO
#define ANALOG_1        86
#define ANALOG_2        87
#define ANALOG_3        88
#define ANALOG_4        89
#define ANALOG_5        90
#define ANALOG_6        91


// CPU Temperature Analogue
#define CPU_TEMP        94  // CPU on chip Tempeature Fix not working TD 6-8-2017 assigned 2 different pin numbers


// Vehicle Voltage Analogue
#define V_SENSE         92


//CURRENT SENSE Power Supply
#define I_SENSE_EN      75  // 12VIO_EN enable to allow the Current sensing for 12VIO
#define I_SENSE_INT     76  // Interupt from Power supply Overcurrent
#define I_SENSE         93  // Analogue AD8 Input for 12VIO current sensing
#define I_SENSE_DAC     95  // DAC Analogue output from CPU to Comparator for Over Current Sensing


// SD
#define SD_SW           36  // SD Card Inserted
#define MCCK            37
#define MCCDA           38
#define MCDA0           39
#define MCDA1           40
#define MCDA2           41
#define MCDA3           42


// CAN
#define CANRX0          69
#define CANTX0          70
#define CAN0_CS         71
#define HS_CS         CAN0_CS

#define CANRX1          72
#define CANTX1          73
#define CAN1_CS         74
#define MS_CS        CAN1_CS


// J1850
#define J1850_PWM_VPW   50
#define J1850_PWM_RX    51
#define J1850_VPW_RX    52
#define J1850P_TX       53
#define J1850N_TX       54


// Power Supply
#define PS_BUCK         48
#define BUCK_DIS        PS_BUCK
#define PS_J1850_9141   49


// XBEE
#define XBEE_RX         0
#define XBEE_TX         1
#define XBEE_RTS        2
#define XBEE_CTS        3
#define XBEE_RST        4
#define XBEE_STAT       5
#define XBEE_VREF       6
#define XBEE_PWM        7
#define XBEE_MULT1      8
#define XBEE_MULT2      9
#define XBEE_MULT3      10
#define XBEE_MULT4      11
#define XBEE_MULT5      12
#define XBEE_MULT6      13
#define SPI0_CS0        47
#define SPI0_CS         SPI0_CS0


// SPI
#define SPI0_MISO       43
#define SPI0_MOSI       44
#define SPI0_CLK        45
#define SPI0_CS1        46


// 9141/LIN
#define LIN_KTX         55
#define LIN_KRX         56
#define LIN_KSLP        57
#define LIN_LTX         58
#define LIN_LRX         59
#define LIN_LSLP        60


// Single Wire Can SWC
#define SWC_RX0         61
#define SWC_RX1         62
//#define SPI0_CS           63
#define SPI0_CS3        63
#define SWC_M0          64
#define SWC_M1          65
#define SWC_CLK         66
#define SWC_INT         67
#define SWC_SOF         68
#define SWC_RST         -1

// **************************************************************************** //

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define SPI_INTERFACE          SPI0
#define SPI_INTERFACE_ID      ID_SPI0
#define SPI_CHANNELS_NUM        4
#define PIN_SPI_SS0             47
#define PIN_SPI_SS1             46
#define PIN_SPI_SS2             103
#define PIN_SPI_SS3             63
#define PIN_SPI_MOSI            44
#define PIN_SPI_MISO            43
#define PIN_SPI_SCK             45
#define BOARD_SPI_SS0           68
#define BOARD_SPI_SS1           52
#define BOARD_SPI_SS2           83
#define BOARD_SPI_SS3        PIN_SPI_SS3
#define BOARD_SPI_DEFAULT_SS BOARD_SPI_SS3


// ** TODO Check all SPI Board to Pin assignments may need to fix this ** TD 6-8-2017//
#define BOARD_PIN_TO_SPI_PIN(x) \
    (x==BOARD_SPI_SS0 ? PIN_SPI_SS0 : \
    (x==BOARD_SPI_SS1 ? PIN_SPI_SS1 : \
    (x==BOARD_SPI_SS2 ? PIN_SPI_SS2 : PIN_SPI_SS3 )))
#define BOARD_PIN_TO_SPI_CHANNEL(x) \
    (x==BOARD_SPI_SS0 ? 0 : \
    (x==BOARD_SPI_SS1 ? 1 : \
    (x==BOARD_SPI_SS2 ? 2 : 3)))

static const uint8_t SS   = BOARD_SPI_SS0;
static const uint8_t SS1  = BOARD_SPI_SS1;
static const uint8_t SS2  = BOARD_SPI_SS2;
static const uint8_t SS3  = BOARD_SPI_SS3;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         81
#define PIN_WIRE_SCL         82
#define WIRE_INTERFACE       TWI1
#define WIRE_INTERFACE_ID    ID_TWI1
#define WIRE_ISR_HANDLER     TWI1_Handler
#define WIRE_ISR_ID          TWI1_IRQn
static const uint8_t SDA1 = PIN_WIRE_SDA;
static const uint8_t SCL1 = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA        79
#define PIN_WIRE1_SCL        80
#define WIRE1_INTERFACE      TWI0
#define WIRE1_INTERFACE_ID   ID_TWI0
#define WIRE1_ISR_HANDLER    TWI0_Handler
#define WIRE1_ISR_ID         TWI0_IRQn

static const uint8_t SDA0  = PIN_WIRE1_SDA;
static const uint8_t SCL0  = PIN_WIRE1_SCL;

// UART3 PINS
#define TXD3                77
#define RXD3                78

/*
 * UART/USART Interfaces
 */
// Serial
#define PINS_UART            98     // XBEE_RX, XBEE_TX
// Serial1
#define PINS_USART0          99     // LIN_KTX, LIN_RTX
// Serial2
#define PINS_USART1          100    // LIN_LTX, LIN_LRX
// Serial3
#define PINS_USART3          101    // UART3/TXD3, RXD3 26 pin connector

/*
 * USB Interfaces
 */
#define PINS_USB             102

/*
 * Analog pins
 */
static const uint8_t A0 = 86;
static const uint8_t A1 = 87;
static const uint8_t A2 = 88;
static const uint8_t A3 = 89;
static const uint8_t A4 = 90;
static const uint8_t A5 = 91;

static const uint8_t A6 = 92;   // VSense
static const uint8_t A7 = 93;   // I_Sense
static const uint8_t A8 = 94;   // CPU on chip Temperature

static const uint8_t A9 = 95;
static const uint8_t DAC1 = 95; //I_Sense_DAC


static const uint8_t CANRX = 69;
static const uint8_t CANTX = 70;

#define ADC_RESOLUTION      12

/*
 * Complementary CAN pins
 */
static const uint8_t CAN1RX = 104;
static const uint8_t CAN1TX = 105;

// CAN0
#define PINS_CAN0            106
// CAN1
#define PINS_CAN1            107


/*
 * DACC
 */
#define DACC_INTERFACE      DACC
#define DACC_INTERFACE_ID   ID_DACC
#define DACC_RESOLUTION     12
#define DACC_ISR_HANDLER    DACC_Handler
#define DACC_ISR_ID         DACC_IRQn

/*
 * PWM
 */
#define PWM_INTERFACE       PWM
#define PWM_INTERFACE_ID    ID_PWM
#define PWM_FREQUENCY       1000
#define PWM_MAX_DUTY_CYCLE  255
#define PWM_MIN_DUTY_CYCLE  0
#define PWM_RESOLUTION      8

/*
 * TC
 */
#define TC_INTERFACE        TC0
#define TC_INTERFACE_ID     ID_TC0
#define TC_FREQUENCY        1000
#define TC_MAX_DUTY_CYCLE   255
#define TC_MIN_DUTY_CYCLE   0
#define TC_RESOLUTION       8

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern UARTClass Serial;    // XBEE_RX, XBEE_TX
extern USARTClass Serial1;  // LIN_KTX, LIN_RTX
extern USARTClass Serial2;  // LIN_LTX, LIN_LRX
extern USARTClass Serial3;  // UART3/TXD3, RXD3 26 pin connector

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial3     // UART3/TXD3, RXD3 26 pin connector
#define SERIAL_PORT_HARDWARE_OPEN1  Serial3     // UART3/TXD3, RXD3 26 pin connector
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3     // UART3/TXD3, RXD3 26 pin connector
#define SERIAL_PORT_HARDWARE        Serial      // XBEE_RX, XBEE_TX
#define SERIAL_PORT_HARDWARE1       Serial1     // LIN_KTX, LIN_RTX
#define SERIAL_PORT_HARDWARE2       Serial2     // LIN_LTX, LIN_LRX
#define SERIAL_PORT_HARDWARE3       Serial3     // UART3/TXD3, RXD3 26 pin connector

#endif /* _VARIANT_MACCHINA_M2_ */

