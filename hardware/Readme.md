Arduino Hardware porting
========

Porting Arduino to a new SAM processor? This should help you!

The Arduino Due is a board built on the Sam3x8e processor. The sam3x8e a very capable processor, but at $13.50 in small quantities it is too expensive for many low cost board designs.

But Arduino is an excellent tool for building hardware that other people can write software for. Porting Arduino to a new board means solving some problems once so that your customers can just focus on their application. You fight code so your customers don't have to.

The official Arduino 1.5.x code repository contains chip definitions for many atmel Sam3 and Sam4 processors, not just the Sam3x8e used in the Due board. With some work, you should be able to make a library file that allows the Arduino IDE to compile code for your own board based on a Sam3 or Sam4 processor.

This code repository is a clone of the Arduino 1.5.x branch, with some additions to allow compiling code for Flutter, a new low cost Arduino-compatible board with a long range wireless radio and a Sam3s1a processor. Compared to the Due's $13.50 processor, the Sam3s1c is a steal at just $2.60 each in quantity. And it still has USB for programming, eliminating the need for an external programming chip. And you still get a powerful 32 bit processor running at 64 Megahertz - much more powerful than the 8 bit 16MHz processors used on most Arduino boards of the past. Porting Arduino to your favorite ARM processor is a great way to advance the state of Open Source Hardware and give your customers powerful hardware for their applications.

Getting Started
==

New ARM-based Arduino boards are defined inside of their own folder called a "variant". There is a variant for the Arduino Due called arduino_due_x located in the folder /hardware/arduino/sam/variants.

To make your own variant folder, just copy an existing variant folder into the variants directory and change its contents to match your hardware. Mostly you should just need to change the variant.cpp and variant.h files, but you will also need to modify the makefiles to compile for your processor. You will mostly likely get code compilation errors, and you will need to figure those out for yourself.

Once your code compiles appropriately, the build system will drop a library file for your board into the Arduino core library folder, and the IDE should be capable of building code for your board. In order to be able to select your board in the Arduino menu, you will also need to change the contents of the boards.txt file in the variants folder.


Lets say your variant is called myboard.

the basic files that you need to customize would be:

/hardware/arduino/sam/variants/myboard/variant.cpp
/hardware/arduino/sam/variants/myboard/variant.h

/hardware/arduino/sam/variants/myboard/build_gcc/Makefile
/hardware/arduino/sam/variants/myboard/build_gcc/libvariant_myboard.mk

A full list of changed files can be found with a folder diff program, such as Meld for linux, which will show you everything that is different between two variant folders.


Debugging
==
As you work to port the code to your board, you probably will not get everything working on your first try, and you will need to debug your code as it runs on the processor. For this you need a hardware debugger such as the Segger J-Link, and a debugging program such as GDB, the default GNU debugger. The Arduino IDE normally compiles against release libraries, which strips out information useful for debugging. If you try to debug your program using an .elf file made by the IDE, it won't be very helpful. To make debugging easier, there is a Makefile and test sketch in the main hardware folder that allows you to build arduino code for your processor at the command line. This is currently configured for testing flutter code, but can be modified to compile code for any variant. It builds the debug version of code so your debugger has access to debugging symbols. There is also a .gdbinit file, which will be automatically loaded when GDB is run from that folder.


More
==
This process can be complex, and there is not a lot of information online as of this writing, but the hope is that this file and the accompanying code will help others in their efforts. For more information plase see the Arduino Developers mailing list.