CC=arm-none-eabi-gcc
MACH=cortex-m33
INC=-I..\common\cmsis -I..\common\st -I.\inc
CFLAGS= -g3 -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0 $(INC) -DSTM32F767xx
LDFLAGS= -nostdlib -T ..\common\scripts\stm32f7_linker.ld -Xlinker -Map=timers.map
SRCDIR=.\src
STADIR=..\common\startup

all:$(SRCDIR)\main.o  $(SRCDIR)\gpio.o $(SRCDIR)\led.o $(STADIR)\stm32f7_startup.o gpio.elf

main.o:$(SRCDIR)\main.c
	$(CC) $(CFLAGS) -o $@ $^

tim.o:$(SRCDIR)\gpio.c
	$(CC) $(CFLAGS) -o $@ $^

led.o:$(SRCDIR)\led.c
	$(CC) $(CFLAGS) -o $@ $^

stm32f7_startup.o:$(STADIR)\stm32f7_startup.c
	$(CC) $(CFLAGS) -o $@ $^

gpio.elf:$(SRCDIR)\main.o $(SRCDIR)\gpio.o $(SRCDIR)\led.o $(STADIR)\stm32f7_startup.o
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	del $(SRCDIR)\*.o $(STADIR)\*.o *.elf

load:
	openocd -f board/st_nucleo_f7.cfg

