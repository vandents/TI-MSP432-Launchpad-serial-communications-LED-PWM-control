/* ----------------------------------------------------------------------------------
* Author: Scott VandenToorn
* Date: April 4, 2019
* Program: MSP432 Launchpad Serial Input LED Intensity Control
* Description: Changes the onboard tri-color LED PWM base on serial input.
*
* PINOUT:
*      P2.0, 2.1, 2.2       Red LED (TA0.1 PWM), Green LED (TA0.2 PWM), Blue LED (TA0.3 PWM)
* ----------------------------------------------------------------------------------- */

#include "msp.h"
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 100

char INPUT_BUFFER[BUFFER_SIZE];
char incomingString[10];
uint8_t storage_location = 0;
uint8_t read_location = 0;

void portRemap(void);               //Remap LED pins to Timer A's, Red -> A0.1, Green -> A0.2, Blue A0.3
void TimerA01init(uint16_t duty);   //Timer A0.1 PWM (Red LED)
void TimerA02init(uint16_t duty);   //Timer A0.2 PWM (Green LED)
void TimerA03init(uint16_t duty);   //Timer A0.3 PWM (Blue LED)
void red(uint16_t dutyR);           //Sets duty cycle for Red LED
void green(uint16_t dutyG);         //Sets duty cycle for Green LED
void blue(uint16_t dutyB);          //Sets duty cycle for Blue LED
void UART0_init(void);              //Initialize serial communications
void serialPrint(char *string);     //Prints string over UART
void readInput(char *string);       //Reads in a line of serial characters
uint16_t calculateDuty(void);       //Function to calculate duty cycle from the incoming string

void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     //Stop watchdog timer
    INPUT_BUFFER[0] = '\0';     //Initialize first spot
    incomingString[0] = '\0';   //Initialize first spot

    uint8_t badString = 0;
    uint16_t dutyCycle = 0;

    /* Remap LED pins to output of different Timer A's
       Red (P2.0) -> A0.1, Green (P2.1) -> A0.1, Blue (P2.2) -> A0.2 */
    portRemap();

    TimerA01init(dutyCycle);    //Timer A0.1 PWM (Red LED)
    TimerA02init(dutyCycle);    //Timer A0.2 PWM (Green LED)
    TimerA03init(dutyCycle);    //Timer A0.3 PWM (Blue LED)

    UART0_init();               //Initialize serial communications

    while(1){
        readInput(incomingString);
        if(incomingString[0] != '\0'){
            /* Only perform further calculations if string is 4 characters */
            if(strlen(incomingString) == 4){
                dutyCycle = calculateDuty();

                /* Flag incoming string as bad if duty cycle > 100 */
                if(dutyCycle > 100)
                    badString = 1;

                /* Flag incoming string as bad if first letter not RGB */
                if(!(incomingString[0] == 'R' | incomingString[0] == 'G' | incomingString[0] == 'B'))
                    badString = 1;

                /* Set duty cycle of corresponding LED if string is valid */
                if(badString == 0){
                    if(incomingString[0] == 'R'){
                        red(dutyCycle);
                    }
                    if(incomingString[0] == 'G'){
                        green(dutyCycle);
                    }
                    if(incomingString[0] == 'B'){
                        blue(dutyCycle);
                    }
                }

                /* Print the validity of the string to the serial port */
                if(badString){
                    serialPrint("Invalid ");
                    serialPrint(incomingString);
                    serialPrint("\n");
                    serialPrint("\n");
                }
                else{
                    serialPrint("Valid ");
                    serialPrint(incomingString);
                    serialPrint("\n");
                    serialPrint("\n");
                }

                badString = 0;              //Reset badString flag
                incomingString[0] = '\0';   //Reset incoming string

            }
        }
    }
}

/* Initialize serial communications */
void UART0_init(void){
    EUSCI_A0->CTLW0 |= 1;       //Put in reset mode for configuration
    EUSCI_A0->MCTLW = 0;        //Disable oversampling
    EUSCI_A0->CTLW0 = 0x0081;   //1 stop bit, no parity, SMCLK, 8-bit data
    EUSCI_A0->BRW = 313;        //(3,000,000 / 9600) = 312.5
    P1->SEL0 |= BIT2 | BIT3;    //P1.2 and P1.3 for UART
    P1->SEL1 &= ~(BIT2 | BIT3);
    EUSCI_A0->CTLW0 &= ~1;      //Take UART out of reset mode

    EUSCI_A0->IFG &= ~BIT0;     //Clear interrupt
    EUSCI_A0->IE |= BIT0;       //Enable interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);
}

/* Prints string over UART */
void serialPrint(char *string){
    int i = 0;                          //Reset i

    while(string[i] != '\0'){
        EUSCI_A0->TXBUF = string[i];    //Send one string character to transmit buffer
        i++;
        while(!(EUSCI_A0->IFG & BIT1));
    }
}

/* Reads in a line of serial data */
void readInput(char *string){
    int i = 0;

    do{ //If a new line hasn't been found yet, but we are caught up to what has been received, wait here for new data
        while(read_location == storage_location && INPUT_BUFFER[read_location] != '\n');

        string[i] = INPUT_BUFFER[read_location];    //Save character into string
        INPUT_BUFFER[read_location] = '\0';

        i++;
        read_location++;

        if(read_location == BUFFER_SIZE)    //If the end of INPUT_BUFFER has been reached, reset read location
            read_location = 0;

    }while(string[i-1] != '\n');    //If a \n was just read, break out of the while loop

    string[i-1] = '\0';             //Replace the \n with a \0 to end the string when returning this function
}

/* Serial Communication ISR */
void EUSCIA0_IRQHandler(void){
    if (EUSCI_A0->IFG & BIT0){      //Interrupt on the receive line
        INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF;   //Store the new piece of data at the present location in the buffer
        EUSCI_A0->IFG &= ~BIT0;     //Clear the interrupt flag right away in case new data is ready

        storage_location++;         //Update to the next position in the buffer

        if(storage_location == BUFFER_SIZE)     //If the end of the buffer was reached, loop back to the start
            storage_location = 0;
    }
}

/* Function to calculate duty cycle from the incoming string */
uint16_t calculateDuty(void){
    int i, dutyCycle;
    uint16_t dutyArray[10];
    dutyArray[0] = 0;
    dutyArray[1] = '\0';

    for(i = 1; i < 4; i++){
        if(incomingString[i] == '0')
            dutyArray[i] = 0;
        else if(incomingString[i] == '1')
            dutyArray[i] = 1;
        else if(incomingString[i] == '2')
            dutyArray[i] = 2;
        else if(incomingString[i] == '3')
            dutyArray[i] = 3;
        else if(incomingString[i] == '4')
            dutyArray[i] = 4;
        else if(incomingString[i] == '5')
            dutyArray[i] = 5;
        else if(incomingString[i] == '6')
            dutyArray[i] = 6;
        else if(incomingString[i] == '7')
            dutyArray[i] = 7;
        else if(incomingString[i] == '8')
            dutyArray[i] = 8;
        else if(incomingString[i] == '9')
            dutyArray[i] = 9;
    }

    dutyCycle = ((dutyArray[1] * 100) + (dutyArray[2] * 10) + (dutyArray[3]));
    return dutyCycle;
}

/* Sets duty cycle for Red LED */
void red(uint16_t dutyR){
    TIMER_A0->CCR[1] = dutyR * 10;
}

/* Sets duty cycle for Green LED */
void green(uint16_t dutyG){
    TIMER_A0->CCR[2] = dutyG * 10;
}

/* Sets duty cycle for Blue LED */
void blue(uint16_t dutyB){
    TIMER_A0->CCR[3] = dutyB * 10;
}

/* Remap Red LED P2.0 to Timer A0.1 output */
/* Remap Green LED P2.1 to Timer A0.2 output */
/* Remap Blue LED P2.2 to Timer A0.3 output */
void portRemap(void){
    PMAP->KEYID = 0x2D52;           //Unlock port mapping controller
    P2MAP->PMAP_REGISTER0 = 20;     //Map P2.0 to Timer A0.1
    P2MAP->PMAP_REGISTER1 = 21;     //Map P2.1 to Timer A0.2
    P2MAP->PMAP_REGISTER2 = 22;     //Map P2.2 to Timer A0.3

    /* Set P2.0 to take Timer A0.1 output */
    /* Set P2.1 to take Timer A0.2 output */
    /* Set P2.2 to take Timer A0.3 output */
    P2->DIR |= BIT0 | BIT1 | BIT2;
    P2->SEL0 |= BIT0 | BIT1 | BIT2;
    P2->SEL1 &= ~(BIT0 | BIT1 | BIT2);

    /* Lock port mapping controller */
    PMAP->CTL = 0;
    PMAP->KEYID = 0;
}

/* Initializes Timer A0.1 PWM with the duty cycle its given (Red LED) */
void TimerA01init(uint16_t dutyR){
    TIMER_A0->CCR[0] = 1000 - 1;    //Period, (clock cycle period * (1000 - 1))
    TIMER_A0->CCR[1] = dutyR * 10;  //dutyR = 1%'s of duty cycle
    TIMER_A0->CCTL[1] = 0xE0;       //CCR0 reset/set mode = 7
    TIMER_A0->CTL = 0x0214;         //Use SMCLK, count up, clear TA0R register
}

/* Initializes Timer A0.2 PWM with the duty cycle its given (Green LED) */
void TimerA02init(uint16_t dutyG){
    TIMER_A0->CCR[0] = 1000 - 1;    //Period, (clock cycle period * (1000 - 1))
    TIMER_A0->CCR[2] = dutyG * 10;  //dutyG = 1%'s of duty cycle
    TIMER_A0->CCTL[2] = 0xE0;       //CCR1 reset/set mode = 7
    TIMER_A0->CTL = 0x0214;         //Use SMCLK, count up, clear TA0R register
}

/* Initialized Timer A0.3 PWM with the duty cycle its given (Blue LED) */
void TimerA03init(uint16_t dutyB){
    TIMER_A0->CCR[0] = 1000 - 1;    //Period, (clock cycle period * (1000 - 1))
    TIMER_A0->CCR[3] = dutyB * 10;  //dutyB = 1%'s of duty cycle
    TIMER_A0->CCTL[3] = 0xE0;       //CCR4 reset/set mode = 7
    TIMER_A0->CTL = 0x0214;         //Use SMCLK, count up, clear TA0R register
}
