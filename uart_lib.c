/*
 * Hersh Joshi 2018
 * UCLA Electrical Engineering Class of 2021
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "uart_lib.h"

/****************************************************************************/
/*                      GLOBAL VARIABLES                                    */
/****************************************************************************/
int RX_value = -1;
unsigned char TX_value = 0;

/****************************************************************************/
/*                      LOCAL FUNCTION DEFINITIONS                          */
/****************************************************************************/

char buffArr[200];
int buffInd=0;

/**
 * Used to synchronize the UART timing on the Arduino and OMAP-L138
 **/
static void UART_synchronization(void)
{
    TX_value = 255;
    while(ReadRX() != 255);
    TX_value = 0;
}

void WriteTX(unsigned char transmit_val)
{
    isTXActive = 1;
    TX_value = transmit_val;
}

short int ReadRX(void)
{
    return RX_value;
}

void Configuration(void)
{
    unsigned int intFlags = 0;
    unsigned int config = 0;

    LCDK_GPIO_init();
    LCDK_SWITCH_init();

    /* Enabling the PSC for UART2.*/
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_UART1, PSC_POWERDOMAIN_ALWAYS_ON,
             PSC_MDCTL_NEXT_ENABLE);

    /* Setup PINMUX */
    UARTPinMuxSetup(1, FALSE);

    /* Enabling the transmitter and receiver*/
    UARTEnable(SOC_UART_1_REGS);

    /* 1 stopbit, 8-bit character, no parity */
    config = UART_WORDL_8BITS;

    /* Configuring the UART parameters*/
    UARTConfigSetExpClk(SOC_UART_1_REGS, SOC_UART_1_MODULE_FREQ,
                        9600, config,
                        UART_OVER_SAMP_RATE_16);

    /* Enabling the FIFO and flushing the Tx and Rx FIFOs.*/
    UARTFIFOEnable(SOC_UART_1_REGS);

    /* Setting the UART Receiver Trigger Level*/
    UARTFIFOLevelSet(SOC_UART_1_REGS, UART_RX_TRIG_LEVEL_1);

    /*
    ** Enable AINTC to handle interrupts. Also enable IRQ interrupt in ARM
    ** processor.
    */
    SetupInt();

    /* Configure AINTC to receive and handle UART interrupts. */
    ConfigureIntUART();

    /* Preparing the 'intFlags' variable to be passed as an argument.*/
    intFlags |= (UART_INT_LINE_STAT  |  \
                 UART_INT_TX_EMPTY |    \
                 UART_INT_RXDATA_CTI);

    /* Enable the Interrupts in UART.*/
    UARTIntEnable(SOC_UART_1_REGS, intFlags);

    UART_synchronization(); //delay(2000);

    WriteTX(0);
}

/*
** \brief   Interrupt Service Routine(ISR) to be executed on UART interrupts.
**          Depending on the source of interrupt, this
**          1> writes to the serial communication console, or
**          2> reads from the serial communication console, or
**          3> reads the byte in RBR if receiver line error has occured.
*/

static void UARTIsr(void)
{
    unsigned char rxData = 0;
    unsigned int int_id = 0;

    /* This determines the cause of UART2 interrupt.*/
    int_id = UARTIntStatus(SOC_UART_1_REGS);

#ifdef _TMS320C6X
    // Clear UART2 system interrupt in DSPINTC
    IntEventClear(SYS_INT_UART1_INT);
#else
    /* Clears the system interrupt status of UART2 in AINTC. */
    IntSystemStatusClear(SYS_INT_UARTINT1);
#endif

    /* Checked if the cause is transmitter empty condition.*/
    if(UART_INTID_TX_EMPTY == int_id) //second argument is new
    {
        if(isTXActive == 1)
        {
            /* Write a byte into the THR if THR is free. */
            UARTCharPutNonBlocking(SOC_UART_1_REGS, TX_value);
        }
    }

    /* Check if the cause is receiver data condition.*/
    if(UART_INTID_RX_DATA == int_id)
    {
        RX_value = rxData = UARTCharGetNonBlocking(SOC_UART_1_REGS);
        if(buffInd<200)
            buffArr[buffInd++]=rxData;
    }

    /* Check if the cause is receiver line error condition.*/
    if(UART_INTID_RX_LINE_STAT == int_id)
    {
        while(UARTRxErrorGet(SOC_UART_1_REGS))
        {
            /* Read a byte from the RBR if RBR has data.*/
            UARTCharGetNonBlocking(SOC_UART_1_REGS);
        }
    }

    return;
}

/*
** \brief   This function invokes necessary functions to configure the ARM
**          processor and ARM Interrupt Controller(AINTC) to receive and
**          handle interrupts.
*/
static void SetupInt(void)
{
#ifdef _TMS320C6X
    // Initialize the DSP INTC
    IntDSPINTCInit();

    // Enable DSP interrupts globally
    IntGlobalEnable();
#else
    /* Initialize the ARM Interrupt Controller(AINTC). */
    IntAINTCInit();

    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Enable the interrupts in GER of AINTC.*/
    IntGlobalEnable();

    /* Enable the interrupts in HIER of AINTC.*/
    IntIRQEnable();
#endif
}

/*
** \brief  This function configures the AINTC to receive UART interrupts.
*/
static void ConfigureIntUART(void)
{
#ifdef _TMS320C6X
    //IntRegister(C674X_MASK_INT4, UARTIsr);
    IntRegister(C674X_MASK_INT4, UARTIsr);
    IntEventMap(C674X_MASK_INT4, SYS_INT_UART1_INT);
    IntEnable(C674X_MASK_INT4);
#else
    /* Registers the UARTIsr in the Interrupt Vector Table of AINTC. */
    IntRegister(SYS_INT_UARTINT1, UART1Isr);

    /* Map the channel number 2 of AINTC to UART2 system interrupt. */
    IntChannelSet(SYS_INT_UARTINT1, 4);

    IntSystemEnable(SYS_INT_UARTINT1);
#endif
}

/****************************END OF FILE*************************************/
