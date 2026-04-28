# target chip
MCU     = atmega328p
# CPU clock in Hz (used by _delay_ms etc.)
F_CPU   = 16000000UL
# base name for build outputs (main.elf, main.hex, ...)
TARGET  = main
# serial port the Arduino bootloader is on
PORT    = /dev/cu.usbserial-210
# bootloader baud rate (optiboot on Uno = 115200)
BAUD    = 115200

# AVR C compiler
CC      = avr-gcc
# extracts/converts sections from ELF (used to make .hex)
OBJCOPY = avr-objcopy
# disassembler — used to produce the .lss listing
OBJDUMP = avr-objdump
# reports flash/SRAM usage of the ELF
SIZE    = avr-size
# flashing tool
AVRDUDE = avrdude

# -mmcu: target chip   -DF_CPU: clock macro for delay headers
# -Os:   optimize for size   -Wall -Wextra: warnings on
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra

# default goal: build the flashable hex
all: $(TARGET).hex

# compile + link C source into an ELF executable
$(TARGET).elf: $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET).elf $(TARGET).c

# convert ELF → Intel HEX (the format avrdude flashes); strip .eeprom section
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $(TARGET).elf $(TARGET).hex

# generate listing file: disassembly interleaved with C source (-S) plus section headers (-h)
$(TARGET).lss: $(TARGET).elf
	$(OBJDUMP) -h -S $(TARGET).elf > $(TARGET).lss

# print flash/RAM usage summary in AVR-friendly format
size: $(TARGET).elf
	$(SIZE) --format=avr --mcu=$(MCU) $(TARGET).elf

# upload the hex over USB serial via the Arduino bootloader
flash: $(TARGET).hex
	$(AVRDUDE) -c arduino -p m328p -P $(PORT) -b $(BAUD) -U flash:w:$(TARGET).hex:i

# remove all generated build artifacts
clean:
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).lss

# targets that aren't real files (so make won't get confused if a file by that name exists)
.PHONY: all flash size clean
