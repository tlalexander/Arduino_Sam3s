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

#define ARDUINO_MAIN
#include "Arduino.h"

/*
 * Cortex-M3 Systick IT handler
 */

extern void SysTick_Handler( void )
{
  // Increment tick count each ms
  TimeTick_Increment() ;
}


/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	init();

	delay(1);

#if defined(USBCON)
	USBDevice.attach();
#endif

	setup();

	for (;;)
	{
		loop();
		if (serialEventRun) serialEventRun();
	}

	return 0;
}


//#include <../arduino/sam/libraries/SPI/SPI.h>


void setup() {
  // put your setup code here, to run once:
    // initialize the digital pin as an output.
  pinMode(PIN_LED, OUTPUT); 
  pinMode(PIN_LED2, OUTPUT); 
  pinMode(PIN_LED3, OUTPUT);  
  pinMode(20, OUTPUT); 
  digitalWrite(20,HIGH);
 // SPI.begin(BOARD_SPI_DEFAULT_SS);
  
  Serial.begin(115200);
  Serial.println("Hello");
}




void loop() {
  // put your main code here, to run repeatedly:
 //Serial.print("led is ");
 //Serial.println(ledstate);
 digitalWrite(PIN_LED3,HIGH);
 delay(250);
 digitalWrite(PIN_LED3,LOW);
 Serial.println("Hello");
 //byte response = SPI.transfer(BOARD_SPI_DEFAULT_SS, 0xFF);
 delay(250);
 
 //digitalWrite(19,LOW); 
// delay(1000);
}
extern "C" void __cxa_pure_virtual() {while (true);}
