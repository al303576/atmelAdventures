/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
/// \dir "Basic PWM2 Project"
///
/// !!!Purpose
///
/// This example demonstrates a simple configuration of three PWM channels to
/// generate variable duty cycle signals. The update of the duty cycle values
/// is made automatically by the Peripheral DMA Controller (PDC).
/// This will cause one or more LEDs the evaluation kit to glow repeatedly.
///
/// !!!See
/// - pwmc: Pulse width modulation controller driver
///
/// !!!Requirements
///
/// This package can be used with all Atmel evaluation kits that have PWMC2
/// interface.
///
/// !!!Description
///
/// Three PWM channels (channel #0, #1 and #3) are configured to generate
/// a 50Hz PWM signal. The update of the duty cycle values is made
/// automatically by the PDC.
///
/// When launched, this program displays a menu on the DBGU, enabling the user
/// to choose between several options:
///   - Set update period for syncronous channel
///   - Set dead time
///   - Set output override
///
/// On most Atmel evaluation kits, PWM signals for channels #0 and #1 are connected
/// to two LEDs on the board. Since the duty cycle of the PWM signals varies continuously,
/// the two LEDs will alternate between their minimum and maximum intensity. Note that
/// on some boards, only one LED will be connected.
///
/// !!!Usage
///
/// -# Build the program and download it inside the evaluation board. Please refer to the
/// <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6132.pdf">
/// SAM-BA User Guide</a>, the
/// <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6132.pdf">
/// GNU-Based Software Development</a> application note or to the
/// <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6132.pdf">
/// IAR EWARM User Guide</a>, depending on your chosen solution.
/// -# Optionally, on the computer, open the DBGU port with a terminal application
///    (e.g. HyperTerminal on Microsoft Windows) with these settings:
///   - 115200 bauds
///   - 8 bits of data
///   - No parity
///   - 1 stop bit
///   - No flow control
/// -# Start the application.
/// -# Depending on the board being used, either one or three LEDs will start glowing repeatedly.
/// -# Select one or more options to set the configuration of PWM channel.
///
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// This file contains all the specific code for the basic-pwm2-project
///
/// !Contents
/// The code can be roughly broken down as follows:
///    - Interrupt handler for the PWM controller
///    - The main function, which implements the program behavior
///
/// Please refer to the list of functions in the #Overview# tab of this unit
/// for more detailed information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <dbgu/dbgu.h>
#include <pwmc/pwmc2.h>
#include <utility/trace.h>

#include <stdio.h>

//------------------------------------------------------------------------------
//         Local definitions
//------------------------------------------------------------------------------

/// PWM frequency in Hz.
#define PWM_FREQUENCY               50

/// Maximum duty cycle value.
#define MAX_DUTY_CYCLE              50
#define MIN_DUTY_CYCLE              0

/// Duty cycle buffer length for three channels
#define DUTY_BUFFER_LENGTH          (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE + 1) * 3

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------

/// Pio pins to configure.
static const Pin pins[] = {
    PINS_DBGU,
    PIN_PWM_LED0,
    PIN_PWM_LED1,
    PIN_PWM_LED2,
    PIN_PWMC_PWML0,
    PIN_PWMC_PWML1,
    PIN_PWMC_PWML2
};

/// duty cycle buffer for PDC transfer
unsigned short dutyBuffer[DUTY_BUFFER_LENGTH];

//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Wait time in ms
//------------------------------------------------------------------------------
void UTIL_Loop(unsigned int loop)
{
    while(loop--);	
}


void UTIL_WaitTimeInMs(unsigned int mck, unsigned int time_ms)
{
    register unsigned int i = 0;
    i = (mck / 1000) * time_ms;
    i = i / 3;
    UTIL_Loop(i);
}

//------------------------------------------------------------------------------
/// Wait time in us
//------------------------------------------------------------------------------
void UTIL_WaitTimeInUs(unsigned int mck, unsigned int time_us)
{
    volatile unsigned int i = 0;
    i = (mck / 1000000) * time_us;
    i = i / 3;
    UTIL_Loop(i);
}

//------------------------------------------------------------------------------
/// Interrupt handler for the PWM controller.
//------------------------------------------------------------------------------
void PWM_IrqHandler(void)
{
    unsigned int isr2 = AT91C_BASE_PWMC->PWMC_ISR2;

    if ((isr2 & AT91C_PWMC_ENDTX) == AT91C_PWMC_ENDTX) {

        PWMC_WriteBuffer(AT91C_BASE_PWMC, dutyBuffer, DUTY_BUFFER_LENGTH);
    }
}

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Display menu
//-----------------------------------------------------------------------------
void DisplayMenu(void)
{
    printf("\n\r");
    printf("===============================================================\n\r");
    printf("Menu: press a key to change the configuration.\n\r");
    printf("===============================================================\n\r");
    printf("  u : Set update period for syncronous channel \n\r");
    printf("  d : Set dead time\n\r");
    printf("  o : Set output override\n\r");
    printf("\n\r");
}

//-----------------------------------------------------------------------------
/// Get 2 digit numkey
/// \return numkey value
//-----------------------------------------------------------------------------
unsigned int GetNumkey2Digit(void)
{
    unsigned int numkey;
    unsigned char key1, key2;

    printf("\n\rEnter 2 digits : ");
    key1 = DBGU_GetChar();
    printf("%c", key1);
    key2 = DBGU_GetChar();
    printf("%c", key2);
    printf("\n\r");

    numkey = (key1 - '0')*10 + (key2 - '0');

    return numkey;
}

//------------------------------------------------------------------------------
/// Outputs a PWM on LED1 & LED2 & LED3 to makes it fade in repeatedly.
/// Channel #0, #1, #2 are linked together as synchronous channels, so they have
/// the same source clock, the same period, the same alignment and
/// are started together. The update of the duty cycle values is made
/// automatically by the Peripheral DMA Controller (PDC).
//------------------------------------------------------------------------------
int main(void)
{
    unsigned int i;
    unsigned char key;
    unsigned int numkey;

    PIO_Configure(pins, PIO_LISTSIZE(pins));
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- Basic PWMC2 Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    UTIL_WaitTimeInMs(BOARD_MCK, 1000);
    UTIL_WaitTimeInUs(BOARD_MCK, 1000);

    // Enable PWMC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;

    // Set clock A to run at PWM_FREQUENCY * MAX_DUTY_CYCLE (clock B is not used)
    PWMC_ConfigureClocks(PWM_FREQUENCY * MAX_DUTY_CYCLE, 0, BOARD_MCK);

    // Configure PWMC channel for LED0 (left-aligned, enable dead time generator)
    PWMC_ConfigureChannelExt(CHANNEL_PWM_LED0, AT91C_PWMC_CPRE_MCKA, 0, 0,
                             0, AT91C_PWMC_DTE, 0, 0);
    PWMC_SetPeriod(CHANNEL_PWM_LED0, MAX_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_PWM_LED0, MIN_DUTY_CYCLE);
    PWMC_SetDeadTime(CHANNEL_PWM_LED0, 5, 5);

    // Configure PWMC channel for LED1
    PWMC_ConfigureChannelExt(CHANNEL_PWM_LED1, AT91C_PWMC_CPRE_MCKA, 0, 0,
                             0, 0, 0, 0);
    PWMC_SetPeriod(CHANNEL_PWM_LED1, MAX_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_PWM_LED1, MIN_DUTY_CYCLE);

    // Configure PWMC channel for LED2
    PWMC_ConfigureChannelExt(CHANNEL_PWM_LED2, AT91C_PWMC_CPRE_MCKA, 0, 0,
                             0, 0, 0, 0);
    PWMC_SetPeriod(CHANNEL_PWM_LED2, MAX_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_PWM_LED2, MIN_DUTY_CYCLE);

    // Set channel #0, #1 and #2 as synchronous channels, update mode = 2
    PWMC_ConfigureSyncChannel(AT91C_PWMC_SYNC0 | AT91C_PWMC_SYNC1 | AT91C_PWMC_SYNC2,
        AT91C_PWMC_UPDM_MODE2, 0, 0);

    // Set Synchronous channel update period value
    PWMC_SetSyncChannelUpdatePeriod(AT91C_PWMC_UPVUPDAL);

    // Configure interrupt for PDC transfer
    IRQ_ConfigureIT(AT91C_ID_PWMC, 0, PWM_IrqHandler);
    IRQ_EnableIT(AT91C_ID_PWMC);
    PWMC_EnableIt(0, AT91C_PWMC_ENDTX);

    // Enable syncronous channels by enable channel #0
    PWMC_EnableChannel(CHANNEL_PWM_LED0);

    // Set override value to 1 on PWMH0, others is 0.
    PWMC_SetOverrideValue(AT91C_PWMC_OOVH0);

    // Fill duty cycle buffer for channel #1, #2 and #3
    // For Channel #1, #2, duty cycle from MIN_DUTY_CYCLE to MAX_DUTY_CYCLE
    // For Channel #3, duty cycle from MAX_DUTY_CYCLE to MIN_DUTY_CYCLE
    for (i = 0; i < DUTY_BUFFER_LENGTH/3; i++) {
        dutyBuffer[i*3] = (i + MIN_DUTY_CYCLE);
        dutyBuffer[i*3+1] = (i + MIN_DUTY_CYCLE);
        dutyBuffer[i*3+2] = (MAX_DUTY_CYCLE - i);
    }
    // Start PDC transfer
    PWMC_WriteBuffer(AT91C_BASE_PWMC, dutyBuffer, DUTY_BUFFER_LENGTH);

    while (1) {
        DisplayMenu();
        key = DBGU_GetChar();

        switch (key) {
            case 'u':
                printf("Input update period between %d to %d.\n\r",
                    0, AT91C_PWMC_UPVUPDAL);
                numkey = GetNumkey2Digit();
                if(numkey <= AT91C_PWMC_UPVUPDAL) {

                    // Set synchronous channel update period value
                    PWMC_SetSyncChannelUpdatePeriod(numkey);
                    printf("Done\n\r");
                } else {

                    printf("Invalid input\n\r");
                }
                break;
            case 'd':
                printf("Input dead time between %d to %d.\n\r",
                    MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
                numkey = GetNumkey2Digit();
                if(numkey >= MIN_DUTY_CYCLE && numkey <= MAX_DUTY_CYCLE) {

                    // Set synchronous channel update period value
                    PWMC_SetDeadTime(CHANNEL_PWM_LED0, numkey, numkey);
                    // Update synchronous channel
                    PWMC_SetSyncChannelUpdateUnlock();
                    printf("Done\n\r");
                } else {

                    printf("Invalid input\n\r");
                }
                break;
            case 'o':
                printf("0: Disable override output on channel #0\n\r");
                printf("1: Enable override output on channel #0\n\r");
                key = DBGU_GetChar();

                if (key == '1') {

                    PWMC_EnableOverrideOutput(AT91C_PWMC_OSSUPDH0 | AT91C_PWMC_OSSUPDL0, 1);
                    printf("Done\n\r");
                } else if (key == '0') {

                    PWMC_DisableOverrideOutput(AT91C_PWMC_OSSUPDH0 | AT91C_PWMC_OSSUPDL0, 1);
                    printf("Done\n\r");
                }
                break;
            default:
                printf("Invalid input\n\r");
                break;
        }
    }
}

