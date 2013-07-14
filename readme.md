Arduino for Sam3s
=================

This is a project to get Arduino working with the Sam3s4 CPU and the other CPUs in that family.
Currently we are tagreting the Olimex SAM3-P256 development kit.

the /build/macosx/work folder contains a runnable macos .app, which can be used to test current functionality.
There is also a mac alias to that in the main folder.

===Building
there are some simple build scripts in the main directory, they just need the paths changed. Each numbered script is just the build command for each step, so that should probably just be put into

build.sh builds the whole chain, though build4.sh may need to be re-run after manually copying the file listed in the Todo, if that issue is still present

==Todo:
The file libvariant_sam3s_ek_gcc_rel.a may not be properly getting copied from:
/hardware/arduino/sam/variants
to:
/hardware/arduino/sam/cores/arduino/
It may not be an issue anymore


Original build instructions
===========================

(these were the basic build instructions I worked from. I think the paths are correct, but we should clean this up. -TLA)

1- Path to GCC ARM toolchain

Set into environment variables the ARM_GCC_TOOLCHAIN variable:
ex:
ARM_GCC_TOOLCHAIN=C:\CodeSourcery_2011.03-42\bin

2- Compile libsam (at91sam peripheral drivers)

Go to:
*****
/Users/taylor/Dropbox/ArduinoDev/workspace/hardware/arduino/sam/system/libsam/build_gcc
*****

run 'make'

This will compile the libsam library and deliver to main Arduino folder the files:

hardware/sam/cores/arduino/libsam_sam3s4c_gcc_dbg.a
hardware/sam/cores/arduino/libsam_sam3s4c_gcc_dbg.a.txt (result of nm)

3- Compile libarduino (Arduino API)

Go to hardware/sam/cores/arduino/build_gcc

run the command: 'cs-make'

This will compile the libarduino library and deliver to main Arduino folder the files:

hardware/sam/cores/arduino/libarduino_sam3s_ek_gcc_dbg.a
hardware/sam/cores/arduino/libarduino_sam3s_ek_gcc_dbg.a.txt (result of nm)

4- Compile libvariant (variant specific library, use Arduino API and libsam)

Go to hardware/sam/variants/sam3s-ek/build_gcc

run the command: 'cs-make'

This will compile the libvariant library and deliver to main Arduino folder the files:

hardware/sam/cores/arduino/libvariant_sam3s_ek_gcc_dbg.a
hardware/sam/cores/arduino/libvariant_sam3s_ek_gcc_dbg.a.txt (result of nm)

5- Compile test application

Go to hardware/sam/cores/arduino/validation/build_gcc

run the command: 'cs-make'

This will compile the test application and deliver the binary into:

hardware/sam/cores/arduino/validation/debug_sam3s_ek/test_gcc_dbg.elf
hardware/sam/cores/arduino/validation/debug_sam3s_ek/test_gcc_dbg.bin
hardware/sam/cores/arduino/validation/debug_sam3s_ek/test_gcc_dbg.map (mapping matching linker script)
hardware/sam/cores/arduino/validation/debug_sam3s_ek/test_gcc_dbg.elf.txt (result of nm)







Original Arduino readme
-----------------------

Arduino is an open-source physical computing platform based on a simple i/o
board and a development environment that implements the Processing/Wiring
language. Arduino can be used to develop stand-alone interactive objects or
can be connected to software on your computer (e.g. Flash, Processing, MaxMSP).
The boards can be assembled by hand or purchased preassembled; the open-source
IDE can be downloaded for free.

For more information, see the website at: http://www.arduino.cc/
or the forums at: http://arduino.cc/forum/

To report a bug in the software, go to:
http://github.com/arduino/Arduino/issues

For other suggestions, use the forum:
http://arduino.cc/forum/index.php/board,21.0.html

INSTALLATION
Detailed instructions are in reference/Guide_Windows.html and
reference/Guide_MacOSX.html.  For Linux, see the Arduino playground:
http://www.arduino.cc/playground/Learning/Linux

CREDITS
Arduino is an open source project, supported by many.

The Arduino team is composed of Massimo Banzi, David Cuartielles, Tom Igoe,
Gianluca Martino, Daniela Antonietti, and David A. Mellis.

Arduino uses the GNU avr-gcc toolchain, avrdude, avr-libc, and code from
Processing and Wiring.

Icon and about image designed by ToDo: http://www.todo.to.it/

