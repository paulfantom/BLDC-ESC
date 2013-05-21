CC=avr-gcc
CFLAGS=-std=c99 -Os -mmcu=attiny861 -Wall
OBJCPY=avr-objcopy
PROG=avrdude

all: main.hex

clean:
	rm -f main.hex main.elf main.o

main.hex: main.elf
	$(OBJCPY) -j .text -j .data -O ihex main.elf main.hex

main.elf: main.o
	$(CC) -o main.elf main.o
  
.o:
	$(CC) $(CFLAGS) -c $*.c

flash: main.hex
	$(PROG) -c usbtiny -p t861 -U flash:w:main.hex

fuse:
	$(PROG) -c usbtiny -p t861 -U lfuse:w:0xcf:m -U hfuse:w:0xdf:m
