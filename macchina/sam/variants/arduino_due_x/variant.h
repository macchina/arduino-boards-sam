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

#ifndef _VARIANT_ARDUINO_DUE_X_
#define _VARIANT_ARDUINO_DUE_X_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			84000000

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
#define PINS_COUNT           (79u)
#define NUM_DIGITAL_PINS     (66u)
#define NUM_ANALOG_INPUTS    (12u)
#define analogInputToDigitalPin(p)  ((p < 12u) ? (p) + 54u : -1)

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
//							M2 PIN Definitions									//
// **************************************************************************** //

// M2 LEDs
#define DS2			32	// (RED)
#define DS3			94	// (YELLOW)
#define DS4			27	// (YELLOW)
#define	DS5			24	// (YELLOW)
#define	DS6			23	// (GREEN)

#define DS7_RED		11	// RGB Red LED
#define	DS7_GREEN	12	// RGB Green LED
#define	DS7_BLUE	5	// RGB Blue LED

#define RGB_RED	= DS7_RED		// RGB Red LED
#define	RGB_GREEN = DS7_GREEN	// RGB Green LED
#define	RGB_BLUE = DS7_BLUE		// RGB Blue LED

/*
#define PIN_LED_13           (13u)
#define PIN_LED_RXL          (72u)
#define PIN_LED_TXL          (73u)
#define PIN_LED              PIN_LED_13
#define PIN_LED2             PIN_LED_RXL
#define PIN_LED3             PIN_LED_TXL
#define LED_BUILTIN          13
*/

// M2 GPIO
#define	GPIO1		35
#define	GPIO2		37
#define	GPIO3		39
#define	GPIO4		41
#define	GPIO5		95
#define	GPIO6		44

// M2 GPIO_Enable
#define	GPIO1_EN	34
#define	GPIO2_EN	36
#define	GPIO3_EN	38
#define	GPIO4_EN	40
#define	GPIO5_EN	9
#define	GPIO6_EN	8

// M2 GPIO PWM Source or SINK
	// by selecting source or sink mode first
	// then PWM the GPIOx_EN enable PIN
#define	GPIO1_PWM	=	GPIO1_EN
#define	GPIO2_PWM	=	GPIO2_EN
#define	GPIO3_PWM	=	GPIO3_EN
#define	GPIO4_PWM	=	GPIO4_EN
#define	GPIO5_PWM	=	GPIO5_EN
#define	GPIO6_PWM	=	GPIO6_EN

// M2 Alternate GPIO PWM
	// by enabling the GPIO_EN pin first
	// then PWM the GPIOx pin
#define	GPIO1_APWM	=	GPIO1
#define	GPIO2_APWM	=	GPIO2
#define	GPIO3_APWM	=	GPIO3
#define	GPIO4_APWM	=	GPIO4
#define	GPIO5_APWM	=	GPIO5
#define	GPIO6_APWM	=	GPIO6

// M2 User Buttons
#define Button1		92
#define Button2		93

// M2 Analogue GPIO
#define	ANALOG_1	64
#define	ANALOG_2	63		
#define	ANALOG_3	61
#define	ANALOG_4	59
#define	ANALOG_5	60
#define	ANALOG_6	54

// CPU Temperature
#define	CPU_TEMP	96	// CPU Chip Temperature

// Vehicle Voltage
#define	V_SENSE		58

// SD
#define	SD_SW		72
#define	MCCK		42
#define	MCCDA		43
#define	MCDA0		73
#define	MCDA1		57
#define	MCDA2		56
#define	MCDA3		55

// CURRENT SENSE
#define	I_SENSE_EN		6	// 12VIO_EN enable the Current sensing for 12VIO
#define	I_SENSE			62	// Analogue AD8 Input for 12VIO current sensing
#define	Over_Current	26
#define	DAC_Sense		67	// DAC output from CPU to Comparator for Over Current Sensing

// CAN
#define	CANTXO		69
#define	CANRX0		68
#define	CAN0_CS		28
#define	HS_CS	CAN0_CS
#define	CANTX1		53
#define	CANRX1		66
#define	CAN1_CS		25
#define	MS_CS	CAN1_CS

// J1850
#define	J1850_PWM_VPW	97
#define	J1850_PWM_RX	3
#define	J1850_VPW_RX	4
#define	J1850P_TX		45
#define	J1850N_TX		7

// Power Supply
#define	PS_Buck			98
#define	PS_J1850_9141	99

// XBEE
#define	XBEE_RX			0
#define	XBEE_TX			1
#define	SPI0_MISO		74
#define	SPIO_MOSI		75
#define	SPIO_CLK		76
#define	SPIO_CS			77
#define	XBEE_RST		100
#define	XBEE_PWM		101
#define	XBEE_MULT1		51
#define	XBEE_MULT2		48
#define	XBEE_MULT3		46
#define	XBEE_MULT4		29
#define	XBEE_MULT5		30
#define	XBEE_MULT6		31
#define XBEE_CTS		22
#define	XBEE_STAT		50
#define	XBEE_VREF		49
#define	XBEE_RTS		2

// 9141/LIN
#define	LIN_KTX			18
#define	LIN_KRX			19
#define	LIN_KSLP		102
#define	LIN_LTX			16
#define	LIN_LRX			17
#define	LIN_LSLP		103

// Single Wire Can SWC
#define	SWC_M0			13
#define	SWC_M1			104
#define	SWC_SOF			10
#define	SWC_CLK			105
#define	SWC_RST			-1
#define	SPIO_CS3		78
#define	SWC_INT			47
#define	SWC_RX0			106
#define	SWC_RX1			107

// **************************************************************************** //

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define SPI_INTERFACE        SPI0
#define SPI_INTERFACE_ID     ID_SPI0
#define SPI_CHANNELS_NUM 4
#define PIN_SPI_SS0          (77u)
#define PIN_SPI_SS1          (87u)
#define PIN_SPI_SS2          (86u)
#define PIN_SPI_SS3          (78u)
#define PIN_SPI_MOSI         (75u)
#define PIN_SPI_MISO         (74u)
#define PIN_SPI_SCK          (76u)
#define BOARD_SPI_SS0        (10u)
#define BOARD_SPI_SS1        (4u)
#define BOARD_SPI_SS2        (52u)
#define BOARD_SPI_SS3        PIN_SPI_SS3
#define BOARD_SPI_DEFAULT_SS BOARD_SPI_SS3

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

#define PIN_WIRE_SDA         (20u)
#define PIN_WIRE_SCL         (21u)
#define WIRE_INTERFACE       TWI1
#define WIRE_INTERFACE_ID    ID_TWI1
#define WIRE_ISR_HANDLER     TWI1_Handler
#define WIRE_ISR_ID          TWI1_IRQn

#define PIN_WIRE1_SDA        (70u)
#define PIN_WIRE1_SCL        (71u)
#define WIRE1_INTERFACE      TWI0
#define WIRE1_INTERFACE_ID   ID_TWI0
#define WIRE1_ISR_HANDLER    TWI0_Handler
#define WIRE1_ISR_ID         TWI0_IRQn

static const uint8_t SDA  = PIN_WIRE_SDA;
static const uint8_t SCL  = PIN_WIRE_SCL;
static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

/*
 * UART/USART Interfaces
 */
// Serial
#define PINS_UART            (81u)
// Serial1
#define PINS_USART0          (82u)
// Serial2
#define PINS_USART1          (83u)
// Serial3
#define PINS_USART3          (84u)

/*
 * USB Interfaces
 */
#define PINS_USB             (85u)

/*
 * Analog pins
 */
static const uint8_t A0  = 54;
static const uint8_t A1  = 55;
static const uint8_t A2  = 56;
static const uint8_t A3  = 57;
static const uint8_t A4  = 58;
static const uint8_t A5  = 59;
static const uint8_t A6  = 60;
static const uint8_t A7  = 61;
static const uint8_t A8  = 62;
static const uint8_t A9  = 63;
static const uint8_t A10 = 64;
static const uint8_t A11 = 65;
static const uint8_t A15 = 95;	// CPU on chip Tempeature
static const uint8_t DAC0 = 66;
static const uint8_t DAC1 = 67;
static const uint8_t CANRX = 68;
static const uint8_t CANTX = 69;

#define ADC_RESOLUTION		12

/*
 * Complementary CAN pins
 */
static const uint8_t CAN1RX = 88;
static const uint8_t CAN1TX = 89;

// CAN0
#define PINS_CAN0            (90u)
// CAN1
#define PINS_CAN1            (91u)


/*
 * DACC
 */
#define DACC_INTERFACE		DACC
#define DACC_INTERFACE_ID	ID_DACC
#define DACC_RESOLUTION		12
#define DACC_ISR_HANDLER    DACC_Handler
#define DACC_ISR_ID         DACC_IRQn

/*
 * PWM
 */
#define PWM_INTERFACE		PWM
#define PWM_INTERFACE_ID	ID_PWM
#define PWM_FREQUENCY		1000
#define PWM_MAX_DUTY_CYCLE	255
#define PWM_MIN_DUTY_CYCLE	0
#define PWM_RESOLUTION		8

/*
 * TC
 */
#define TC_INTERFACE        TC0
#define TC_INTERFACE_ID     ID_TC0
#define TC_FREQUENCY        1000
#define TC_MAX_DUTY_CYCLE   255
#define TC_MIN_DUTY_CYCLE   0
#define TC_RESOLUTION		8

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern UARTClass Serial;
extern USARTClass Serial1;
extern USARTClass Serial2;
extern USARTClass Serial3;

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
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3

#endif /* _VARIANT_ARDUINO_DUE_X_ */

