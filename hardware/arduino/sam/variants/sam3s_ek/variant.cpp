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
 *   0             |  PA8   | "RX0"
 *   1             |  PA9   | "TX0"
 *   2       TIOA0 |  PB25  |
 *   3       TIOA7 |  PC28  |
 *   4       NPCS1 |  PA29  |
 *           TIOB6 |  PC26  |
 *   5       TIOA6 |  PC25  |
 *   6       PWML7 |  PC24  |
 *   7       PWML6 |  PC23  |
 *   8       PWML5 |  PC22  |
 *   9       PWML4 |  PC21  |
 *  10       NPCS0 |  PA28  |
 *           TIOB7 |  PC29  |
 *  11       TIOA8 |  PD7   |
 *  12       TIOB8 |  PD8   |
 *  13       TIOB0 |  PB27  | LED AMBER "L"
 *  14       TXD3  |  PD4   | "TX3"
 *  15       RXD3  |  PD5   | "RX3"
 *  16       TXD1  |  PA13  | "TX2"
 *  17       RXD1  |  PA12  | "RX2"
 *  18       TXD0  |  PA11  | "TX1"
 *  19       RXD0  |  PA10  | "RX1"
 *  20             |  PB12  | "SDA"
 *  21             |  PB13  | "SCL"
 *  22             |  PB26  |
 *  23             |  PA14  |
 *  24             |  PA15  |
 *  25             |  PD0   |
 *  26             |  PD1   |
 *  27             |  PD2   |
 *  28             |  PD3   |
 *  29             |  PD6   |
 *  30             |  PD9   |
 *  31             |  PA7   |
 *  32             |  PD10  |
 *  33             |  PC1   |
 *  34             |  PC2   |
 *  35             |  PC3   |
 *  36             |  PC4   |
 *  37             |  PC5   |
 *  38             |  PC6   |
 *  39             |  PC7   |
 *  40             |  PC8   |
 *  41             |  PC9   |
 *  42             |  PA19  |
 *  43             |  PA20  |
 *  44             |  PC19  |
 *  45             |  PC18  |
 *  46             |  PC17  |
 *  47             |  PC16  |
 *  48             |  PC15  |
 *  49             |  PC14  |
 *  50             |  PC13  |
 *  51             |  PC12  |
 *  52       NPCS2 |  PB21  |
 *  53             |  PB14  |
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
  { PIOA, PIO_PA9A_URXD0,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // URXD0
  { PIOA, PIO_PA10A_UTXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UTXD0

  // 2
  { PIOA, PIO_PA0B_TIOA0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHA0     }, // TIOA0
  { PIOA, PIO_PA15B_TIOA1,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHA1     }, // TIOA1
  { PIOA, PIO_PA26B_TIOA2,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHA2     }, // TIOA2

  // 5
  { PIOA, PIO_PA1B_TIOB0,    ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHB0     }, // TIOB0
  { PIOA, PIO_PA15C_PWML3,   ID_PIOA, PIO_PERIPH_C, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH3,     NOT_ON_TIMER }, // PWML3
  { PIOA, PIO_PA30A_PWML2,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH2,     NOT_ON_TIMER }, // PWML2
  { PIOA, PIO_PA20B_PWML1,   ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH1,     NOT_ON_TIMER }, // PWML1
  // 9
  { PIOB, PIO_PA16B_TIOB1,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHB1     }, // TIOB1
  { PIOB, PIO_PA27B_TIOB2,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHB2     }, // TIOB2

  { PIOA, PIO_PA1,           ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 11

  // LED2
  { PIOA, PIO_PA17,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 12

  // LED1 (digital 13)
  { PIOA, PIO_PA18,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LED1

  // 14/15 - USART1 (Serial3)
  { PIOA, PIO_PA22A_TXD1,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TXD1
  { PIOA, PIO_PA21A_RXD1,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // RXD1

  // 16/17 - USART0 (Serial2)
  { PIOA, PIO_PA6A_TXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TXD0
  { PIOA, PIO_PA5A_RXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // RXD0

 // 18/19 - UART1 (Serial1)
  { PIOB, PIO_PB3A_UTXD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UTXD1
  { PIOB, PIO_PB2A_URXD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // URXD1

  // 20/21 - TWI1
  { PIOB, PIO_PB4A_TWD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD1 - SDA0
  { PIOB, PIO_PB5A_TWCK1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK1 - SCL0

  { PIOA, PIO_PA2,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 22
  { PIOA, PIO_PA3,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 23
  { PIOA, PIO_PA4,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 24
  { PIOA, PIO_PA5,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 25
  { PIOA, PIO_PA6,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 26
  { PIOA, PIO_PA7,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 27
  { PIOA, PIO_PA8,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 28




  // 29 .. 41 - Analog pins
  // ----------------------
  { PIOB, PIO_PB3X1_AD7,    ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC0,   ADC7,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD0
  { PIOB, PIO_PB2X1_AD6,    ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC1,   ADC6,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD1
  { PIOB, PIO_PB1X1_AD5,    ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC2,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD2
  { PIOB, PIO_PB0X1_AD4,    ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC3,   ADC4,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD3
  // 33
  { PIOA, PIO_PA20X1_AD3,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC4,   ADC3,   NOT_ON_PWM,  NOT_ON_TIMER     }, // AD4
  { PIOA, PIO_PA19X1_AD2,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC5,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER     }, // AD5
  { PIOA, PIO_PA18X1_AD1,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC6,   ADC1,   NOT_ON_PWM,  NOT_ON_TIMER     }, // AD6
  { PIOA, PIO_PA17X1_AD0,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC7,   ADC0,   NOT_ON_PWM,  NOT_ON_TIMER     }, // AD7
  // 37
  { PIOB, PIO_PC13X1_AD10,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC8,   ADC10,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD8
  { PIOB, PIO_PC15X1_AD11,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC9,   ADC11,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD9
  { PIOB, PIO_PC12X1_AD12,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC10,  ADC12,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD10
  { PIOB, PIO_PC29X1_AD13,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC11,  ADC13,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD11
  { PIOB, PIO_PC30X1_AD14,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC12,  ADC14,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD12

  // 42/43 - DAC0/DAC1
  { PIOB, PIO_PB13X1_DAC0,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC13,  DA0,    NOT_ON_PWM,  NOT_ON_TIMER }, // DAC0
  { PIOB, PIO_PB14X1_DAC1,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC14,  DA1,    NOT_ON_PWM,  NOT_ON_TIMER }, // DAC1

  // 44/45 - TWI0
  { PIOA, PIO_PA3A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD0 - SDA1
  { PIOA, PIO_PA4A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK0 - SCL1

  // 46/47 - LEDs
  { PIOA, PIO_PA23,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LED AMBER RXL
  { PIOA, PIO_PA21,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LED AMBER TXL

  // 48/49/50 - SPI
  { PIOA, PIO_PA12A_MISO,     ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MISO
  { PIOA, PIO_PA13A_MOSI,     ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MOSI
  { PIOA, PIO_PA14A_SPCK,     ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPCK

  // 51 - SPI CS0
  { PIOA, PIO_PA11A_NPCS0,    ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS0

  // 52 - SPI CS3 (unconnected)
  { PIOB, PIO_PA22B_NPCS3,    ID_PIOB,PIO_PERIPH_B,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS3

  // 53 .. 58 - "All pins" masks

  // 53 - TWI0 all pins
  { PIOA, PIO_PA3A_TWD0|PIO_PA4A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 54 - TWI1 all pins
  { PIOB, PIO_PB4A_TWD1|PIO_PB5A_TWCK1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 55 - UART0 (Serial) all pins
  { PIOA, PIO_PA9A_URXD0|PIO_PA10A_UTXD0, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 56 - UART1 (Serial1) all pins
  { PIOB, PIO_PB3A_UTXD1|PIO_PB2A_URXD1, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 57 - USART0 (Serial2) all pins
  { PIOA, PIO_PA6A_TXD0|PIO_PA5A_RXD0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 58 - USART1 (Serial3) all pins
  { PIOA, PIO_PA22A_TXD1|PIO_PA21A_RXD1, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  // 59 - USB
  { PIOB, PIO_PB10|PIO_PB11, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ID - VBOF
  // 60, 61
  { PIOA, PIO_PA31A_NPCS1,    ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS3
  { PIOA, PIO_PA10B_NPCS2,    ID_PIOA,PIO_PERIPH_B,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS3



  // END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
RingBuffer rx_buffer1;
RingBuffer rx_buffer2;

UARTClass Serial(UART0, UART0_IRQn, ID_UART0, &rx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }

UARTClass Serial1(UART1, UART1_IRQn, ID_UART1, &rx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }


// IT handlers
void UART0_Handler(void)
{
  Serial.IrqHandler();
}

// IT handlers
void UART1_Handler(void)
{
  Serial1.IrqHandler();
}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */

RingBuffer rx_buffer3;
RingBuffer rx_buffer4;


USARTClass Serial2(USART0, USART0_IRQn, ID_USART0, &rx_buffer3);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }
USARTClass Serial3(USART1, USART1_IRQn, ID_USART1, &rx_buffer4);
void serialEvent3() __attribute__((weak));
void serialEvent3() { }

// IT handlers
void USART0_Handler(void)
{
  Serial2.IrqHandler();
}

void USART1_Handler(void)
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
  for (int i = 0; i < PINS_COUNT; i++)
    digitalWrite(i, LOW);

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port U(S)ART pins
  PIO_Configure(
    g_APinDescription[PINS_UART0].pPort,
    g_APinDescription[PINS_UART0].ulPinType,
    g_APinDescription[PINS_UART0].ulPin,
    g_APinDescription[PINS_UART0].ulPinConfiguration);
  digitalWrite(0, HIGH); // Enable pullup for RX0
  PIO_Configure(
    g_APinDescription[PINS_UART1].pPort,
    g_APinDescription[PINS_UART1].ulPinType,
    g_APinDescription[PINS_UART1].ulPin,
    g_APinDescription[PINS_UART1].ulPinConfiguration);
  digitalWrite(19, HIGH); // Enable pullup for RX1

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

  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pPort,
    g_APinDescription[PINS_USB].ulPinType,
    g_APinDescription[PINS_USB].ulPin,
    g_APinDescription[PINS_USB].ulPinConfiguration);

  

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

