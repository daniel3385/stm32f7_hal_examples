CC=arm-none-eabi-gcc
MACH=cortex-m33
INC=-I..\common_includes\cmsis -I..\common_includes\st -I.\inc
CFLAGS= -g3 -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0 $(INC) -DSTM32F767xx --specs=nano.specs
#LDFLAGS= -nostdlib -T ..\scripts\stm32f7_linker.ld -Xlinker -Map=uart_example.map
LDFLAGS= -mcpu=$(MACH) -T ..\scripts\stm32f7_linker.ld -Xlinker -Map=uart_example.map --specs=nano.specs
SRCDIR=.\src
STADIR=..\startup

all:$(SRCDIR)\main.o $(SRCDIR)\uart.o $(SRCDIR)\led.o $(STADIR)\stm32f7_startup.o $(STADIR)\syscalls.o uart_example.elf

main.o:$(SRCDIR)\main.c
	$(CC) $(CFLAGS) -o $@ $^

uart.o:$(SRCDIR)\uart.c
	$(CC) $(CFLAGS) -o $@ $^

led.o:$(SRCDIR)\led.c
	$(CC) $(CFLAGS) -o $@ $^

syscalls.o:$(STADIR)\syscalls.c
	$(CC) $(CFLAGS) -o $@ $^

stm32f7_startup.o:$(STADIR)\stm32f7_startup.c
	$(CC) $(CFLAGS) -o $@ $^

uart_example.elf:$(SRCDIR)\main.o $(SRCDIR)\uart.o $(SRCDIR)\led.o $(STADIR)\stm32f7_startup.o $(STADIR)\syscalls.o
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	del $(SRCDIR)\*.o $(STADIR)\*.o *.elf

load:
	openocd -f board/st_nucleo_f7.cfg

