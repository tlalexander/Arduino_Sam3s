#*******************************************************
#
#  Connect to J-Link and debug application in flash on SAM3S.
#

# Define 'reset' command
define reset

# Connect to the J-Link gdb server
target remote localhost:2331

# Reset the chip to get to a known state
monitor reset

# Select flash device
monitor flash device = AT91SAM3S1A

# Enable flash download and flash breakpoints
monitor flash download = 1

#file arduino/sam/cores/arduino/validation/build_gcc/debug_flutter/test_gcc_dbg.elf
file build/test.elf
#file /tmp/build6619425255672302645.tmp/Blink.cpp.elf

# Load the program
load

thbreak setup

# Reset peripheral (RSTC_CR)
set *0x400e1400 = 0xA5000004

# Initialize PC and stack pointer
mon reg sp=(0x400000)
#set *0x400004 = *0x400004 & 0xFFFFFFFE
mon reg pc=(0x400004)

info reg

# End of 'reset' command
end

#Define read regs function (rr)
define rr

echo \nreading uart0\n
set $UART = 0x400E0600

echo UART_MR\n
x $UART+0x004
echo UART_IMR\n
x $UART+0x010
echo UART_RHR\n
x $UART+0x018
echo UART_THR\n
x $UART+0x01C
echo UART_SR\n
x $UART+0x014

echo \n\nreading PIOA\n
set $PIOA = 0x400E0E00

echo PIO_PSR - PIO Status Register\n
x $PIOA+0x008
echo PIO_OSR\n
x $PIOA+0x018
echo PIO_ODSR\n
x $PIOA+0x038

echo \n\nreading Systick\n

echo CTRL - SysTick Control Register\n
x 0xE000E010
echo RELOAD - SysTick Reload Value Register\n
x 0xE000E014
echo CURRENT - SysTick Current Value Register\n
x 0xE000E018
echo CALIB - SysTick Calibration Value Register\n
x 0xE000E01C

echo \n\nreading RSTC\n
set $RSTC = 0x400E1400
echo RSTC_SR - Reset Status Register\n
x $RSTC+0x000
echo RSTC_MR - Reset Mode Register\n
x $RSTC+0x008

echo \n\nreading PMC\n
set $PMC = 0x400E0400

echo PMC_SCSR - System Clock Status Register\n
x $PMC+0x0008
echo PMC_PCSR0 - Peripheral Clock Status Register\n
x $PMC+0x0018
echo CKGR_MOR - Main Oscillator Register\n
x $PMC+0x0020
echo CKGR_MCFR - Main Clock Frequency Register\n
x $PMC+0x0024
echo CKGR_PLLAR - PLLA Register\n
x $PMC+0x0028
echo CKGR_PLLBR - PLLB Register\n
x $PMC+0x002C
echo PMC_MCKR - Master Clock Register\n
x $PMC+0x0020
echo PMC_USB - USB Clock Register\n
x $PMC+0x0038
echo PMC_SR - Status Register\n
x $PMC+0x0068
echo PMC_IMR - Interrupt Mask Register\n
x $PMC+0x006C

echo \n\nreading WDT\n
set $WDT = 0x400e1450

echo WDT_MR - Watchdog Timer Mode Register\n
x $PMC+0x0004
echo PMC_SCSR - Watchdog Status Register\n
x $WDT+0x0008

end

define dumpPeriph
# prevent GDB from stopping every screenfull
set height 0
# GDB output is now also copied into gdb.txt
set logging on
x/768xw 0x400E0000
x/448xw 0x400E0E00
quit
end
