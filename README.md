Raspberry Pi Pico Zero USB-UART Bridge
=================================

This program turns the Raspberry Pi Pico Zero into a dual-channel USB-to-UART bridge and adds a blinking WS2812 “traffic” indicator that shows which UART is currently active.

The LED is driven from the on-board PIO state machine, so the blinking does not interfere with UART timing or USB throughput.

Disclaimer
----------

This software is provided without warranty, according to the MIT License, and should therefore not be used where it may endanger life, financial stakes, or cause discomfort and inconvenience to others.

Raspberry Pi Pico Zero Pinout
------------------------

| Rpi Pico Zero | Function |
|:-:|:-:|
| GPIO12 | UART0 TX |
| GPIO13 | UART0 RX |
| GPIO4  | UART1 TX |
| GPIO5  | UART1 RX |
