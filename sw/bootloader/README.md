Memory map
0000'0000h main memory (SPRAM)

0002'0000h boot loader (BRAM)

1000'0000h character based output: write one byte
1000'1000h LEDs (each bit), lowest bit indicates program termination
1000'2000h UART RX
1000'3000h UART TX
1000'4000h UART signal width of one bit in clock cycles