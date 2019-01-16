UART Bootloader
===============

Simple bootloader that can be used as firmware for Danzig. It reads bytes from
a UART and writes them to the main memory, starting at address 0. When the
transmission of the memory image is finished, execution continues at address 0.
The actual data is preceeded by the length of the memory image in bytes, encoded
as an ASCII string followed by a non-numerical ASCII character (e.g. LF 0Ahex).

Example image to write "Hello World!" to address 0:

    31 32 0A 48 65 6C 6C 6F 20 57 6F 6C 64 21  "12 Hello World!"

The code is position independent, therefore it can be executed anywhere within
the address space. It requires only 512 bytes of RAM, but since this memory is
used for code and stack, it must not be read-only. On an FPGA typically a 
pre-initialisable 4KiBit-Block-RAM is used for it.

The communication with the RS232 UART is done via three memory mapped addresses:

|  address   | mapping                                      |
| ---------: | :------------------------------------------- |
| 1000'2000h | UART RX (LSB)                                |
| 1000'3000h | UART TX (LSB)                                |
| 1000'4000h | UART signal width of one bit in clock cycles |

The last value is the length of one bit in clock cycles, i.e. the clock rate
(e.g. 12'000'000 for 12 MHz) divided by the baud rate (e.g. 115'200). It is a
read-only constant provided by the hardware, since it depends on the clock rate.
