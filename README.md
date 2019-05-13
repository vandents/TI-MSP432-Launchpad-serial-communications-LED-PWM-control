# TI-MSP432-Launchpad-serial-communications
This program changes the MSP432 Launchpad's onboard tri-colored LED based on UART serial input from a computer.


The Launchpad takes commands consisting of 4 characters so that the first character represents an LED color (R, G, or B) and the final three characters represent a duty cycle (0 – 100%) of the corresponding LED's PWM (higher duty cycle => brighter LED). Each command ends with a line feed (‘\n’).

If the MSP receives a valid communication (for example R060), it will respond with: 
* valid R060
If the MSP receives an invalid communication (for example R160), it will respond with: 
* invalid R160



The UART communication properties are as follows: 
* Baud rate: 9600, 
* Data: 8-bit, 
* Parity: None, 
* Stop: One bit, 

To make serial communications possible on a MacBook I used a [USB to TTL serial cable](https://www.adafruit.com/product/954 "Adafruit - USB to TTL") and a simple serial port terminal application called [CoolTerm](https://freeware.the-meiers.org "freeware.the-meiers.org").

