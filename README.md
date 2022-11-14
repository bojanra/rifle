# rifle
Control your antenna rotator via the serial interface. The firmware is for the ATMEL AVR processor
without hardware support for serial communication. Therefore the RS232 is implemented via IRQ Bitbanging.

From the beginning the target processor was AT Tiny 261. Due to the larger code, it was moved to AT Tiny 861.

## Compile
just run

      make

## Programming
The default programmer is **usbasp**. Adapt to your environment and run

      make writeflash

When programming for the first time, also update the fuse bits

      make fuses
