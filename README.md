# stm32f7_hal_examples
This repository contains some examples of STM32F7 bare-metal peripheral drivers. Folders explanation below.

common: In this folder there are headers from ST, headers from CMSIS, linkers script and startup files. Some builds use libnano, others don't use it, in order to share the same startup routine I use __LIBNANO__ verification.

gpio: TBD

spi: TBD

timers: Two timers are initialized and used to blink leds. Timer 1 will blink green led every 1 second. Systick timer will blink blue led every 2 seconds.

uart_polling: Opens 115200 8n1 uart using UART3 which uses virtually the usb cable from the board.
A menu will be displayed in this uart where the user can turn on and off the leds of the board. 