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

#ifndef _VARIANT_FLUTTER_
#define _VARIANT_FLUTTER_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			64000000

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
#define PINS_COUNT           (27u)
#define NUM_DIGITAL_PINS     (14u)
#define NUM_ANALOG_INPUTS    (4u)

// Interrupts
#define digitalPinToInterrupt(p)  ((p) < NUM_DIGITAL_PINS ? (p) : -1)

// LEDs
//#define PIN_LED_RXL          (21u)
//#define PIN_LED_TXL          (21u)
#define PIN_LED              (17u)
#define PIN_LED2             (18u)
#define PIN_LED3             (19u)
#define LED_BUILTIN          (17u)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define SPI_INTERFACE        SPI
#define SPI_INTERFACE_ID     ID_SPI
#define SPI_CHANNELS_NUM      4
#define PIN_SPI_SS0          (26u)
//#define PIN_SPI_SS1          (87u)
//#define PIN_SPI_SS2          (86u)
//#define PIN_SPI_SS3          (78u)
#define PIN_SPI_MOSI         (24u)
#define PIN_SPI_MISO         (23u)
#define PIN_SPI_SCK          (25u)

#define BOARD_SPI_SS0        (26u) //TODO: SPI pin number when not being SPI (digital pin number)
//#define BOARD_SPI_SS1        (4u)
//#define BOARD_SPI_SS2        (52u)
//#define BOARD_SPI_SS3        PIN_SPI_SS3
#define BOARD_SPI_DEFAULT_SS BOARD_SPI_SS0

#define BOARD_PIN_TO_SPI_PIN(x) \
	(x==BOARD_SPI_SS0 ? PIN_SPI_SS0 )
  // \
	//(x==BOARD_SPI_SS1 ? PIN_SPI_SS1 : \
	//(x==BOARD_SPI_SS2 ? PIN_SPI_SS2 : PIN_SPI_SS3 )))
#define BOARD_PIN_TO_SPI_CHANNEL(x) \
	(x==BOARD_SPI_SS0 ? 0 )
  //: \
	//(x==BOARD_SPI_SS1 ? 1 : \
	//(x==BOARD_SPI_SS2 ? 2 : 3)))

static const uint8_t SS   = BOARD_SPI_SS0;
//static const uint8_t SS1  = BOARD_SPI_SS1;
//static const uint8_t SS2  = BOARD_SPI_SS2;
//static const uint8_t SS3  = BOARD_SPI_SS3;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (21u)
#define PIN_WIRE_SCL         (22u)
#define WIRE_INTERFACE       TWI0
#define WIRE_INTERFACE_ID    ID_TWI0
#define WIRE_ISR_HANDLER     TWI0_Handler

//#define PIN_WIRE1_SDA        (70u)
//#define PIN_WIRE1_SCL        (71u)
//#define WIRE1_INTERFACE      TWI0
//#define WIRE1_INTERFACE_ID   ID_TWI0
//#define WIRE1_ISR_HANDLER    TWI0_Handler

/*
 * UART/USART Interfaces
 */
// Serial
#define PINS_UART0           (28u) //single line in Pin array that indicates both UART pins
// Serial1
#define PINS_USART1          (29u) //single line in Pin array that indicates both USART pins

/*
 * USB Interfaces
 */
#define PINS_USB             (30u) //single line in Pin array that indicates both USB pins

/*
 * Analog pins
 */
static const uint8_t A0  = 4;
static const uint8_t A1  = 5;
static const uint8_t A2  = 6;
static const uint8_t A3  = 7;

//static const uint8_t DAC0 = 66;
//static const uint8_t DAC1 = 67;
//static const uint8_t CANRX = 68;
//static const uint8_t CANTX = 69;
#define ADC_RESOLUTION		12

/*
 * Complementary CAN pins
 */
//static const uint8_t CAN1RX = 88;
//static const uint8_t CAN1TX = 89;

// CAN0
//#define PINS_CAN0            (90u)
// CAN1
//#define PINS_CAN1            (91u)


/*
 * DACC
 */
//#define DACC_INTERFACE		DACC
//#define DACC_INTERFACE_ID	ID_DACC
//#define DACC_RESOLUTION		12
//#define DACC_ISR_HANDLER    DACC_Handler
//#define DACC_ISR_ID         DACC_IRQn

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

extern UARTClass Serial1;
extern USARTClass Serial;

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
#define SERIAL_PORT_MONITOR          Serial
#define SERIAL_PORT_USBVIRTUAL       SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN    Serial
#define SERIAL_PORT_HARDWARE_OPEN1   Serial1
#define SERIAL_PORT_HARDWARE         Serial
#define SERIAL_PORT_HARDWARE1        Serial1

#endif /* _VARIANT_FLUTTER_ */

