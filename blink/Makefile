RISCV_PATH ?= /home/$(USER)/tools/riscv64-unknown-elf-toolsuite/latest

CC = $(RISCV_PATH)/bin/riscv64-unknown-elf-clang
OC = $(RISCV_PATH)/bin/riscv64-unknown-elf-objcopy
OD = $(RISCV_PATH)/bin/riscv64-unknown-elf-objdump

# C compilation directives
CFLAGS += -c
CFLAGS += -std=c11
CFLAGS += -Wall
CFLAGS += -Os
CFLAGS += --target=riscv32
# QingKe V4B 144MHz
CFLAGS += -march=rv32imac
CFLAGS += -mabi=ilp32

# Linker directives.
LFLAGS += -Wall
LFLAGS += -Wl,--no-relax
LFLAGS += -nostdlib
LFLAGS += -Tscript.ld
LFLAGS += -march=rv32imac
LFLAGS += --target=riscv32

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
upload: $(MAIN).elf
	~/.cargo/bin/wlink flash $(MAIN).elf

.PHONY: all
all: $(MAIN).lst $(MAIN).elf

.PHONY: clean
clean:
	rm -f $(OBJS) *.o *.elf *.lst
