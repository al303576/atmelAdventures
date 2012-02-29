/**************************************************************************
MODULE:    MCOHW
CONTAINS:  Hardware driver specification for MicroCANopen implementation
           The specific implementations are named mcohwXXX.c, where
           XXX represents the CAN hardware used.
COPYRIGHT: Embedded Systems Academy, Inc. 2002-2007.
           All rights reserved. www.microcanopen.com
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
LICENSE:   THIS IS THE EDUCATIONAL VERSION OF MICROCANOPEN
           See file license_educational.txt or
           www.microcanopen.com/license_educational.txt
           A commercial MicroCANopen license is available at
           www.CANopenStore.com
VERSION:   3.30, ESA 30-JAN-07
           $LastChangedDate: 2009-12-10 09:17:17 +0100 (to, 10 dec 2009) $
           $LastChangedRevision: 33668 $
---------------------------------------------------------------------------

Implementation recommendations:

1.) CAN interrupt
The CAN interrupt should check all the possible error flags and set the
global variable gMCOHW_status accordingly. Fatal errors must result in
a call to MCOUSER_FatalError with an error code in the range of 0x8000
to 0x87FF.

If a transmit queue is implemented, the transmit interrupt should be used 
to trigger transmission of the next message in the transmit queue.

On "Basic CAN" controllers the receive interrupt copies the incoming message
into a receive queue. CAN controllers with "Full CAN" style capabilities
or internal receive queue might not need to maintain a software queue.
In case a hardware queue or buffers are used, the interrupt should still
check for a potential overrun and set bit RXOR in gMCOHW_status in case
of an overrun.

2.) Timer interrupt
A 1ms timer interrupt needs to implement a local 1ms UNSIGNED16 timer tick.
The timer tick is only accessable via the functions MCOHW_GetTime and
MCOHW_IsTimeExpired to avoid data inconsistency.

In case only a multiple of 1ms is available on a system, the timer tick
would need to be incremented in each interrupt in a way that the timer
tick is still accurate (for example increment by 4 all 4ms).
***************************************************************************/ 

#include "mco.h"

// Status bits for function MCOHW_GetStatus
#define HW_INIT 0x01
#define HW_CERR 0x02
#define HW_ERPA 0x04
#define HW_RXOR 0x08
#define HW_TXOR 0x10
#define HW_TXBSY 0x40
#define HW_BOFF 0x80

/**************************************************************************
DOES:    This function returns the global status variable.
CHANGES: The status can be changed anytime by this module, for example from 
         within an interrupt service routine or by any of the other 
         functions in this module.
BITS:    0: INIT - set to 1 after a completed initialization
                   left 0 if not yet inited or init failed
         1: CERR - set to 1 if a CAN bit or frame error occured
         2: ERPA - set to 1 if a CAN "error passive" occured
         3: RXOR - set to 1 if a receive queue overrun occured
         4: TXOR - set to 1 if a transmit queue overrun occured
         5: Reserved
         6: Reserved
         7: BOFF - set to 1 if a CAN "bus off" error occured
**************************************************************************/
UNSIGNED8 MCOHW_GetStatus
  (
  void
  );

/**************************************************************************
DOES:    This function implements the initialization of the CAN interface.
RETURNS: 1 if init is completed 
         0 if init failed, bit INIT of MCOHW_GetStatus stays 0
**************************************************************************/
UNSIGNED8 MCOHW_Init 
  (  
  UNSIGNED16 BaudRate   // Allowed values: 1000, 800, 500, 250, 125, 50, 25, 10
  );

/**************************************************************************
DOES:    This function implements the initialization of a CAN ID hardware
         filter as supported by many CAN controllers.
RETURNS: 1 if filter was set 
         2 if this HW does not support filters 
           (in this case HW will receive EVERY CAN message)
         0 if no more filter is available
**************************************************************************/
UNSIGNED8 MCOHW_SetCANFilter 
  (
  UNSIGNED16 CANID      // CAN-ID to be received by filter
  );

/**************************************************************************
DOES:    This function implements a CAN transmit queue. With each
         function call is added to the queue.
RETURNS: 1 Message was added to the transmit queue
         0 If queue is full, message was not added,
           bit TXOR in MCOHW_GetStatus set
NOTES:   The MicroCANopen stack will not try to add messages to the queue
         "back-to-back". With each call to MCO_ProcessStack, a maximum
         of one message is added to the queue. For many applications
         a queue with length "1" will be sufficient. Only applications
         with a high busload or very slow bus speed might need a queue
         of length "3" or more.
**************************************************************************/
UNSIGNED8 MCOHW_PushMessage 
  (
  CAN_MSG MEM_FAR *pTransmitBuf // Data structure with message to be send
  );

/**************************************************************************
DOES:    This function implements a CAN receive queue. With each
         function call a message is pulled from the queue.
RETURNS: 1 Message was pulled from receive queue
         0 Queue empty, no message received
NOTES:   Implementation of this function greatly varies with CAN
         controller used. In an SJA1000 style controller, the hardware
         queue inside the controller can be used as the queue. Controllers
         with just one receive buffer need a bigger software queue.
         "Full CAN" style controllers might just implement multiple
         message objects, one each for each ID received (using function
         MCOHW_SetCANFilter).
**************************************************************************/
UNSIGNED8 MCOHW_PullMessage
  (
  CAN_MSG MEM_FAR *pTransmitBuf // Data structure with message received
  );

/**************************************************************************
DOES:    This function reads a 1 millisecond timer tick. The timer tick
         must be a UNSIGNED16 and must be incremented once per millisecond.
RETURNS: 1 millisecond timer tick
NOTES:   Data consistency must be insured by this implementation.
         (On 8-bit systems, disable the timer interrupt incrementing
         the timer tick while executing this function)
         Systems that cannot provide a 1ms tick may consider incrementing
         the timer tick only once every "x" ms, if the increment is by "x".
**************************************************************************/
UNSIGNED16 MCOHW_GetTime 
  (
  void
  );

/**************************************************************************
DOES:    This function compares a UNSIGNED16 timestamp to the internal timer tick
         and returns 1 if the timestamp expired/passed.
RETURNS: 1 if timestamp expired/passed
         0 if timestamp is not yet reached
NOTES:   The maximum timer runtime measurable is 0x8000 (about 32 seconds).
         For the usage in MicroCANopen that is sufficient. 
**************************************************************************/
UNSIGNED8 MCOHW_IsTimeExpired 
  (
  UNSIGNED16 timestamp // Timestamp to be checked for expiration
  );
/**************************************************************************
// Recommended implementation for this function (8051 version):         
{
UNSIGNED16 time_now;

  EA = 0; // Disable Interrupts
  time_now = gTimCnt;
  EA = 1; // Enable Interrupts
  timestamp++; // To ensure the minimum runtime
  if (time_now > timestamp)
  {
    if ((time_now - timestamp) < 0x8000)
      return 1;
    else
      return 0;
  }
  else
  {
    if ((timestamp - time_now) > 0x8000)
      return 1;
    else
      return 0;
  }
}
**************************************************************************/

/**************************************************************************
DOES:    Initializes access to non-volatile memory.
**************************************************************************/
void NVOL_Init (
  void
  );


/**************************************************************************
DOES:    Reads a data byte from non-volatile memory.
         Must be provided by application driver.
NOTE:    The address is relative, an offset to NVOL_STORE_START
RETURNS: The data read from memory
**************************************************************************/
UNSIGNED8 NVOL_ReadByte (
  UNSIGNED16 address // location of byte in NVOL memory
  );


/**************************************************************************
DOES:    Writes a data byte to non-volatile memory
         Must be provided by application driver.
NOTE:    The address is relative, an offset to NVOL_STORE_START
RETURNS: nothing
**************************************************************************/
void NVOL_WriteByte (
  UNSIGNED16 address, // location of byte in NVOL memory
  UNSIGNED8 dat
  );
