IDIR=./include
LDIR=./lib
OBJ_DIR=./obj
SRC_DIR=./src
DRIVERS_IDIR=./esp-open-sdk/ESP8266_NONOS_SDK-2.1.0-18-g61248df/driver_lib/include
ESPRESSIF_IDIR=./esp-open-sdk/ESP8266_NONOS_SDK-2.1.0-18-g61248df/include
ESPTOOL_IDIR=./esp-open-sdk/esptool/flasher_stub

LDLIBS = -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -lm -lc -lgcc -Wl,--end-group
LDFLAGS = -Teagle.app.v6.ld
MAKEFLAGS += --quiet

LINK_TARGET=CliBox
BINARY=$(LINK_TARGET)-0x00000.bin

CC=xtensa-lx106-elf-gcc	
CFLAGS=-I$(IDIR) -I$(DRIVERS_IDIR) -I$(ESPRESSIF_IDIR)-g -Wall -mlongcalls -std=c99

SRCS=$(wildcard $(SRC_DIR)/*.c)
OBJS=$(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS))

$(BINARY): $(LINK_TARGET)
	esptool.py elf2image $^

# Link #
$(LINK_TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS) $(LDLIBS)

# Compile #
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c $< -o $@ $(CFLAGS)

# Flash #
flash: $(BINARY)
	esptool.py --port /dev/ttyUSB0 write_flash -fm dio 0x00000 /home/kabeibl/Repositories/CliBox/$(LINK_TARGET)-0x00000.bin
	esptool.py --port /dev/ttyUSB0 write_flash -fm dio 0x10000 /home/kabeibl/Repositories/CliBox/$(LINK_TARGET)-0x10000.bin
	make clean

# Erase Flash #
flash_erase:
	esptool.py --port /dev/ttyUSB0 erase_flash

# Clean #
clean:
	rm -rf $(LINK_TARGET)-0x00000.bin $(LINK_TARGET)-0x10000.bin