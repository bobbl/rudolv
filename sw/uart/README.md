UART Bootloader
===============

Simple bootloader that can be used as firmware for RudolV. It reads bytes from
a UART and writes them to the main memory, starting at address 0. When the
transmission of the memory image is finished, execution continues at address 0.
The actual data is preceeded by the length of the memory image in bytes, encoded
as an ASCII string followed by a non-numerical ASCII character (e.g. LF 0Ahex).

Example image to write "Hello World!" to address 0:

    31 32 0A 48 65 6C 6C 6F 20 57 6F 6C 64 21  "12 Hello World!"

The code is position independent, therefore it can be executed anywhere within
the address space.

The large `bootloader` has a prompt and some LED output, but requires about 512
bytes of RAM. Since this memory is used for code and stack, it must not be
read-only. On an FPGA typically a pre-initialisable 4KiBit-Block-RAM is used for
it.

The small `bl` has no prompt and requires less than 100 Bytes of read-only
memory. It should be used when no pre-initialisable Block-RAM is available and
LUTs have to be used to synthesize it (e.g. on Microsemi FPGAs).


UART Test
=========

There is an additional `test` program that can be used as firmware instead of
the bootloader to do an initial test of the UART.


UART Interfaces
===============


Bit banging
-----------

Communication via CSR 07C0hex. Read bit 0 to get the current value of RX. Write
bit 0 to change the value of TX. Bits 31..1 are constant and give the width of
a pulse in cycles (clock frequency divided by baud rate, (e.g. 12'000'000 /
115'200).


Character oriented
------------------

Communication via CSR 0BC0hex. If bit 8 is clear, there is a char in the receive
buffer and its value is in bits 7..0. To remove the char from the buffer, set
bit 8 via CSRRS. To send a char, wait untul bit 9 is clear and use CSRRW to
write the char.


Mi-V compatible
---------------

| mem  addr  | description                                              |
| ---------: | :------------------------------------------------------- |
| 7000'0000h | write here to send a char                                |
| 7000'0004h | read here to receive a char                              |
| 7000'0010h | bit 0 set: ready to send, bit 1 set: ready to read       |
