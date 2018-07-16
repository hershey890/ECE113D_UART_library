/*
 * Hersh Joshi 2018
 * UCLA Electrical Engineering Class of 2021
 *
 * Example code for communicating with the Arduino via UART
 */

#include "uart_lib.h"

/****************************************************************************/
/*                      AVAILABLE FUNCTIONS                                 */
/****************************************************************************/
/**
 * Add at the start of the program to configure the UART connection
 **/
extern void Configuration(void);

/**
 * Transmits an unsigned char on the TX pin, the TX pin transmits
 * 0 by default if this is not called
 **/
extern void WriteTX(unsigned char transmit_val);

/**
* Returns a -1 if no value is read, otherwise returns an unsigned char
* based on what value is read from the RX pin
**/
extern short int ReadRX(void);

int RX_value = 0;

int main(void)
{
    //Example code
    Configuration();

    while(1) {
        /* Default Transmission Value is 0 */
        WriteTX(5);

        /* Will return -1 if nothing is being received */
        RX_value = ReadRX();
    }
}
