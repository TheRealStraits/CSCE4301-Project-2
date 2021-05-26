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

![CubeMX Config](https://github.com/TheRealStraits/CSCE4301-Project-2/blob/main/Diagrams/CubeMX%20Config.PNG?raw=true)

### Hardware Connections

Please follow the following Diagram in order to make the necessary connections with your Thumper.

![Hardware Interface](https://github.com/TheRealStraits/CSCE4301-Project-2/blob/main/Diagrams/Hardware%20Interface.PNG?raw=true)


### Code Upload

There are two C files inside the "Project Files Folder", one contains the main function which includes all of the necessary functions for the timers and all the system logic. The other file is the interrupt handler file that contains the UART1 interrupt handler which is used to retrieve the data from the Bluetooth module. Make sure to copy these two codes in their respective counterparts in the project that you have generated using CubeMX.

If you wish to run the project directly or use the ready-to-go project, then proceed to download the ZIP file containing all of the source files and MDK files. You will find it in the same folder as the main source files mentioned earlier.

Once you are in Keil uVision, make sure to set debugging to ST-Link and the Compiler to Compiler 5. Once all of the necessary compilations have been made and the code is built, you can proceed to upload the image to your STM32L4 Microcontroller and test the Thumper.


##### Extra Notes
This project is part of the CSCE 4301 Embedded Systems course in the American University under the supervision of Dr. Mohamad Shalan.
