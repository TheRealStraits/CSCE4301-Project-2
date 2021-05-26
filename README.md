# CSCE 4301 Project 2

## Use Instructions

Make sure to follow these configuration and installation steps to be able to run the program.

### STM32 CubeMX Configuration

In order to be able to run the necessary hardware, you will need to configure the following peripherals in the CubeMX software : 

- UART2 for issuing commands to the Dagu Thumper and general debugging purposes (baud rate of 115200)
- UART1 for getting commands from the HM-10 Bluetooth Module
- TIM1 as input capture mode in order to calculate how long the ultrasonic sensor echo pin has been high for
- TIM2 to set up a microsecond timer
- Two GPIO output pins (TRIG & LED) in order to trigger the ultrasonic sensor and for debugging
- Finally, set the SYS to Serial Wire

![CubeMX Config]()

