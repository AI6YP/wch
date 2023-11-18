RISCV_PATH ?= /home/$(USER)/tools/riscv64-unknown-elf-toolsuite/latest

CC = $(RISCV_PATH)/bin/riscv64-unknown-elf-clang
OC = $(RISCV_PATH)/bin/riscv64-unknown-elf-objcopy
OD = $(RISCV_PATH)/bin/riscv64-unknown-elf-objdump

ASFLAGS += -c
ASFLAGS += -Os
ASFLAGS += -Wall
ASFLAGS += --target=riscv32
# QingKe V4B 144MHz
ASFLAGS += -march=rv32imac
ASFLAGS += -mabi=ilp32

# C compilation directives
CFLAGS += -c
CFLAGS += -std=c11
LFLAGS += -nostdlib
CFLAGS += -Wall
CFLAGS += -Oz
CFLAGS += --target=riscv32
# QingKe V4B 144MHz
CFLAGS += -march=rv32imac
CFLAGS += -mabi=ilp32
# CFLAGS += -fno-integrated-as

# Linker directives.
LFLAGS += -Wall
LFLAGS += -Wl,--no-relax
LFLAGS += -nostdlib
LFLAGS += -Tscript.ld
LFLAGS += -march=rv32imac
LFLAGS += --target=riscv32

INCLUDE = -I../ch32v20x/EVT/EXAM/SRC/Debug/ \
	-I../ch32v20x/EVT/EXAM/SRC/Peripheral/inc/ \
	-I.

MAIN = blink

AS_SRC = ./startup_ch32v20x_D6.S

C_SRC  = ./$(MAIN).c
C_SRC += ./core_riscv.c
C_SRC += ../ch32v20x/EVT/EXAM/SRC/Peripheral/src/ch32v20x_rcc.c
C_SRC += ../ch32v20x/EVT/EXAM/SRC/Peripheral/src/ch32v20x_gpio.c
C_SRC += ./ch32v20x_it.c
C_SRC += ./system_ch32v20x.c

OBJS  = $(AS_SRC:.S=.o)
OBJS += $(C_SRC:.c=.o)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

%.elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

%.bin: %.elf
	$(OC) -S -O binary $< $@

%.lst: %.elf
	$(OD) --source --all-headers --demangle --line-numbers --wide $< > $@

.PHONY: upload
upload: $(MAIN).elf
	~/.cargo/bin/wlink flash $(MAIN).elf

# .PHONY: watch
# watch: $(MAIN).elf
# 	~/.cargo/bin/wlink flash --enable-sdi-print --watch-serial $(MAIN).elf

# ~/.cargo/bin/wlink flash --enable-sdi-print --watch-serial $(MAIN).elf

.PHONY: all
all: $(MAIN).lst $(MAIN).elf $(MAIN).bin

.PHONY: clean
clean:
	rm -f $(OBJS) *.o *.elf *.lst *.bin
