IDIR=../include
ODIR=obj
LDIR=../lib

LDLIBS = -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -lm -lc -lgcc -lstdc++ -Wl,--end-group
LDFLAGS = -Teagle.app.v6.ld
MAKEFLAGS += --quiet

CC=xtensa-lx106-elf-g++
CFLAGS=-I$(IDIR) -g -Wall -mlongcalls

main-0x00000.bin: main
	esptool.py elf2image $^
	make clean > /dev/null

main: main.o

main.o: main.cpp
	$(CC) -c $< $(CFLAGS)

flash: 0x00000.bin
	esptool.py --port /dev/ttyUSB0 write_flash -fm dout 0 0x00000.bin 0x10000 0x10000.bin
	make clean > /dev/null

clean:
	rm -rf main main.o #main-0x00000.bin main-0x10000.bin