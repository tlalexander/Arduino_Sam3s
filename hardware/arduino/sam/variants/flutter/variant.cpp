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
 * DUE Board pin   |  PORT  | Label
 * ----------------+--------+-------
 *   0             |  PA5   | "RX0"
 *   1             |  PA6   | "TX0"
 *   2             |  PB2   | "RX1"
 *   3             |  PB3   | "TX1"
 *   4             |  PA17  | "A0"
 *   5             |  PA19  | "A1"
 *   6             |  PB0   | "A2"
 *   7             |  PB1   | "A3"
 *   8             |  PA8   | 
 *   9             |  PA7   |
 *  10             |  PA2   |
 *  11             |  PA1   |
 *  12             |  PA0   |
 *  13             |  PA4   |
 *  14             |  PA3   |
 *  15             |  PB12  | "B1"
 *  16             |  PA18  | "B2"
 *  17             |  PA20  | "RGB_R"
 *  18             |  PB5   | "RGB_G"
 *  19             |  PA16  | "RGB_B"
 *  20             |  PB4   | "RGB_RP"
 *  21             |  PB11  | "USB_P"
 *  22             |  PB10  | "USB_M"




 *  54             |  PA16  | "A0"
 *  55             |  PA24  | "A1"
 *  56             |  PA23  | "A2"
 *  57             |  PA22  | "A3"
 *  58       TIOB2 |  PA6   | "A4"
 *  69             |  PA4   | "A5"
 *  60       TIOB1 |  PA3   | "A6"
 *  61       TIOA1 |  PA2   | "A7"
 *  62             |  PB17  | "A8"
 *  63             |  PB18  | "A9"
 *  64             |  PB19  | "A10"
 *  65             |  PB20  | "A11"
 *  66             |  PB15  | "DAC0"
 *  67             |  PB16  | "DAC1"
 *  68             |  PA1   | "CANRX"
 *  69             |  PA0   | "CANTX"
 *  70             |  PA17  | "SDA1"
 *  71             |  PA18  | "SCL1"
 *  72             |  PC30  | LED AMBER "RX"
 *  73             |  PA21  | LED AMBER "TX"
 *  74       MISO  |  PA25  |
 *  75       MOSI  |  PA26  |
 *  76       SCLK  |  PA27  |
 *  77       NPCS0 |  PA28  |
 *  78       NPCS3 |  PB23  | unconnected!
 *
 * USB pin         |  PORT
 * ----------------+--------
 *  ID             |  PB11
 *  VBOF           |  PB10
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
  // 0 .. 53 - Digital pins
  // ----------------------
  // 0/1 - UART (Serial)
  { PIOA, PIO_PA5,      ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,               NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D0  RXD0
  { PIOA, PIO_PA6,      ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D1  TXD0

   // 16/17 - USART1 (Serial2)
  { PIOA, PIO_PB2A_URXD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,           NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D2  UTXD1
  { PIOA, PIO_PB3A_UTXD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,           NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D3  URXD1

  // 4-7 Analog Pins
  { PIOA, PIO_PA17X1_AD0,   ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,             ADC0,   ADC0,   NOT_ON_PWM,  NOT_ON_TIMER }, // D4 AD0
  { PIOA, PIO_PA19X1_WKUP9, ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,             ADC1,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER }, // D5 AD1
  { PIOB, PIO_PB0X1_AD4,    ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,             ADC2,   ADC4,   NOT_ON_PWM,  NOT_ON_TIMER }, // D6 AD2
  { PIOB, PIO_PB1X1_AD5,    ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,             ADC3,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // D7 AD3

  { PIOA, PIO_PA8,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D8
  { PIOA, PIO_PA7,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D9
  { PIOA, PIO_PA2,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D10
  { PIOA, PIO_PA1,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D11
  { PIOA, PIO_PA0,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D12
  { PIOA, PIO_PA4,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D13
  { PIOA, PIO_PA3,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D14
  //Buttons
  { PIOB, PIO_PB12,    ID_PIOB, PIO_INPUT, PIO_DEFAULT, PIN_ATTR_DIGITAL,                    NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D15 B1
  { PIOA, PIO_PA18,    ID_PIOA, PIO_INPUT, PIO_DEFAULT, PIN_ATTR_DIGITAL,                    NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D16 B2
  //LED
  { PIOA, PIO_PA20,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D17 RGB_R
  { PIOB, PIO_PB5,     ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D18 RGB_G
  { PIOA, PIO_PA16,    ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D19 RGB_B
  { PIOB, PIO_PB4,     ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D20 RGB_R_P
  // 21/22 - TWI0
  { PIOA, PIO_PA3A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,            NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D21 TWD0 - SDA1
  { PIOA, PIO_PA4A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,            NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D22 TWCK0 - SCL1
   // 23/24/25 - SPI
  { PIOA, PIO_PA12A_MISO,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D23 MISO
  { PIOA, PIO_PA13A_MOSI,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D24 MOSI
  { PIOA, PIO_PA14A_SPCK,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D25 SPCK
  // 26 - SPI CS0
  { PIOA, PIO_PA11A_NPCS0,ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL,                NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // D26 NPCS0


  // 27...30 - "All pins" masks
  // 27 - TWI0 all pins
  { PIOA, PIO_PA3A_TWD0|PIO_PA4A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 28 - USART0 (Serial) all pins
  { PIOA, PIO_PA5A_RXD0|PIO_PA6A_TXD0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 29 - UART1 (Serial1) all pins
  { PIOB, PIO_PB2A_URXD1|PIO_PB3A_UTXD1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  // 30 - USB
  { PIOB, PIO_PB1X1_AD5, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ID
  
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }

  // END
  
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
RingBuffer rx_buffer1;

UARTClass Serial1(UART1, UART1_IRQn, ID_UART1, &rx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }

// IT handlers
void UART1_Handler(void)
{
  Serial.IrqHandler();
}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
RingBuffer rx_buffer2;

USARTClass Serial(USART0, USART0_IRQn, ID_USART0, &rx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }


// IT handlers
void USART0_Handler(void)
{
  Serial1.IrqHandler();
}



// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
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

  // Disable watchdog
  WDT_Disable(WDT);

  // Initialize C library
  __libc_init_array();

  // Disable pull-up on every pin
  for (uint i = 0u; i < PINS_COUNT; i++)
	  digitalWrite(i, LOW);

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  //PIOC->PIO_OWER = 0xFFFFFFFF;
  //PIOD->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port UART pins
  PIO_Configure(
    g_APinDescription[PINS_UART0].pPort,
    g_APinDescription[PINS_UART0].ulPinType,
    g_APinDescription[PINS_UART0].ulPin,
    g_APinDescription[PINS_UART0].ulPinConfiguration);
    digitalWrite(0, HIGH); // Enable pullup for RX0

  // Initialize Serial port USART pins
  // Pins are disconnected from PIO controller and hooked to the peripheral.
  // Currently PIO_Configure always enables the pullup resistor for peripherals. This appears to be a bug, as it is not written correctly for that purpose, but has that affect.
  PIO_Configure(
    g_APinDescription[PINS_USART1].pPort,
    g_APinDescription[PINS_USART1].ulPinType,
    g_APinDescription[PINS_USART1].ulPin,
    g_APinDescription[PINS_USART1].ulPinConfiguration);
 
/*
TODO: wire up USB ID line and check out USB configuration
  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pPort,
    g_APinDescription[PINS_USB].ulPinType,
    g_APinDescription[PINS_USB].ulPin,
    g_APinDescription[PINS_USB].ulPinConfiguration);

  
//TODO: Initialize I2C pins for crypto IC
  PIO_Configure(
    g_APinDescription[PINS_SPI].pPort,
    g_APinDescription[PINS_SPI].ulPinType,
    g_APinDescription[PINS_SPI].ulPin,
    g_APinDescription[PINS_SPI].ulPinConfiguration);
*/

  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

  // Initialize analogOutput module
  analogOutputInit();
}

#ifdef __cplusplus
}
#endif

