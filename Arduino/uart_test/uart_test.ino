/*
 * Hersh Joshi 2018
 * UCLA Electrical Engineering Class of 2021
 * 
 * Example code for communicating with the OMAP-L138 via UART
 */

/* 
 * SoftwareSerial is the Library used for the Arduino software Serial/UART connection 
 * 
 * Do not use the TX and RX lines on the Arduino as these are linked
 * USB/Serial Connection used to interface with the computer and will
 * interfere with the connection to the computer.
 */
#include "SoftwareSerial.h"

/*
 * Use any digital pins but do not use the actual TX or RX pins
 */
#define RX_pin 10
#define TX_pin 11

int RX_val = 0;

/* Must be used to configure the digital pins for Serial use */
SoftwareSerial mySerial(RX_pin, TX_pin);

/* Used to synchronize the UART timing on the Arduino and OMAP-L138 
 * 
 * Without this function, the Arduino or OMAP-L138 can start reading the signal
 * sent via the UART connection at the wrong point, e.g.
 * UART between Arduino and the OMAP-L138 is idle high, 1-HIGH, 0-LOW
 * 200 to binary is 0b11001000
 *       1 1 0 0 1 0 0 0
 * _ _   _ _     _       _ _ _
 *    |_|   |_ _| |_ _ _|  
 *     !                 ! 
 *  start     bit        stop
 *  bit       word       bit
 *   
 * The start of the binary word is read to be a 0, however, if the Arduino or the OMAP-L138
 * start the binary word late/in the middle of the binary word, it can read the wrong value
 * as the start bit and throw the whole synchronization off (e.g. it can read the first 0 in
 * 11001000 as the start bit instead of the first first stop bit. This function synchronizes the
 * OMAP-L138 and the UART to prevent this from happeneing
 * 
 * 1. The OMAP-L138 and Arduino transmit continuous 255's
 * 2. When the OMAP-L138 reads a 255, it shows that the Arduino's code is up and running
 * 3. The OMAP-L138 sendss over a 0 to signify that it has received a 255
 * 4. The Arduino and OMAP-L138 both leave synchronization loops and transmit a default value
 *    in this case 0
 *    
 * Why 255? the bit value for 255 is 11111111, therefore the only 0 that can be interpreted as the
 * start bit is the actual 0 start bit
 */
void UART_synchronization()
{
  while(mySerial.read() != 0) 
    mySerial.write(255);
}

void setup()
{
  /* Computer to Arduino Serial connection via USB */
  Serial.begin(9600);
  /* Setting up baud rate for Arduino to OMAP-L138 Serial connection */
  mySerial.begin(9600);
  UART_synchronization();
}

void loop()
{
  /* Function used to transmit values */
  mySerial.write(0);
  /* Function used to recieve values */
  RX_val = mySerial.read();
}

