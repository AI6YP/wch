RISCV_PATH ?= /home/$(USER)/tools/riscv64-unknown-elf-toolsuite/latest

CC = $(RISCV_PATH)/bin/riscv64-unknown-elf-gcc
OC = $(RISCV_PATH)/bin/riscv64-unknown-elf-objcopy
OD = $(RISCV_PATH)/bin/riscv64-unknown-elf-objdump

# C compilation directives
CFLAGS += -c
CFLAGS += -std=c11
CFLAGS += -Wall
CFLAGS += -Os
# V2A
CFLAGS += -march=rv32ec
CFLAGS += -mabi=ilp32e

# Linker directives.
LFLAGS += -Wall
LFLAGS += -Wl,--no-relax
LFLAGS += -nostdlib
LFLAGS += -Tscript.ld
LFLAGS += -march=rv32ec

INCLUDE = -I.

MAIN = blink
C_SRC = ./$(MAIN).c

OBJS += $(C_SRC:.c=.o)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

%.elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

%.lst: %.elf
	$(OD) --source --all-headers --demangle --line-numbers --wide $< > $@

.PHONY: upload
upload: $(MAIN).elf $(MAIN).lst
	~/.cargo/bin/wlink flash $(MAIN).elf

.PHONY: all
all: $(MAIN).lst $(MAIN).elf

.PHONY: clean
clean:
	rm -f $(OBJS) *.o *.elf *.lst
