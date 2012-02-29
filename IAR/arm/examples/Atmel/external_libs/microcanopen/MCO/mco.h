/**************************************************************************
MODULE:    MCO
CONTAINS:  MicroCANopen implementation
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
***************************************************************************/ 

#ifndef _MCO_H
#define _MCO_H

#include "nodecfg.h"


/**************************************************************************
DON'T CHANGE ANYTHING IN THIS FILE, 
USE NODECFG.H FOR CUSTOMIZATION
**************************************************************************/

#define TRUE 1;
#define FALSE 0;

/**************************************************************************
CANopen NMT (Network Management) Master Msg and Slave States
**************************************************************************/
#define NMTMSG_OP 1
#define NMTMSG_STOP 2
#define NMTMSG_PREOP 128
#define NMTMSG_RESETAPP 129
#define NMTMSG_RESETCOM 130

#define NMTSTATE_BOOT 0
#define NMTSTATE_STOP 4
#define NMTSTATE_OP 5
#define NMTSTATE_PREOP 127

/**************************************************************************
MACROS FOR OBJECT DICTIONARY ENTRIES
***************************************************************************/ 

#define GETUNSIGNED8(val,pos) ((val >> pos) & 0xFF)
#define GETUNSIGNED8S16(val) GETUNSIGNED8(val, 0), GETUNSIGNED8(val, 8)
#define GETUNSIGNED8S32(val) GETUNSIGNED8(val, 0), GETUNSIGNED8(val, 8), GETUNSIGNED8(val,16), GETUNSIGNED8(val,24)
#define SDOREPLY(index,sub,len,val) 0x43 | ((4-len)<<2), GETUNSIGNED8S16(index), sub, GETUNSIGNED8S32(val)
#define SDOREPLY4(index,sub,len,d1,d2,d3,d4) 0x43 | ((4-len)<<2), GETUNSIGNED8S16(index), sub, d1, d2, d3, d4

#define ODENTRY(index,sub,len,offset) {index, sub, len, offset}


#ifndef USE_EVENT_TIME
  #ifndef USE_INHIBIT_TIME
#error At least one, USE_EVENT_TIME or USE_INHIBIT_TIME must be defined!
  #endif
#endif

#if (NR_OF_RPDOS == 0)
  #if (NR_OF_TPDOS == 0)
#error At least one PDO must be defined!
  #endif
#endif

#if ((NR_OF_TPDOS > 4) || (NR_OF_RPDOS > 4))
#error This configuration was never tested by ESAcademy!
#endif


/**************************************************************************
DEFINITION OF DATA STRUCTURES
**************************************************************************/

// Data structure for a single CAN message 
typedef struct
{
  UNSIGNED16 ID;                 // Message Identifier 
  UNSIGNED8 LEN;                 // Data length (0-8) 
  UNSIGNED8 dummy32bit;          // Fill it to 32-bit width
  UNSIGNED8 BUF[8];              // Data buffer 
} CAN_MSG;

// This structure holds all node specific configuration
typedef struct
{
  CAN_MSG heartbeat_msg;         // Heartbeat message contents
  UNSIGNED16 Baudrate;           // Current Baud rate in kbit
  UNSIGNED16 heartbeat_time;     // Heartbeat time in ms
  UNSIGNED16 heartbeat_timestamp;// Timestamp of last heartbeat
  UNSIGNED8 Node_ID;             // Current Node ID (1-126)
  UNSIGNED8 error_code;          // Bits: 0=RxQueue 1=TxQueue 3=CAN
  UNSIGNED8 error_register;      // Error regiter for OD entry [1001,00]
} MCO_CONFIG;

// This structure holds all the TPDO configuration data for one TPDO
typedef struct 
{
  CAN_MSG CAN;                   // Current/last CAN message to be transmitted
#ifdef USE_EVENT_TIME
  UNSIGNED16 event_time;         // Event timer in ms (0 for COS only operation)
  UNSIGNED16 event_timestamp;    // If event timer is used, this is the 
                                 // timestamp for the next transmission
#endif
#ifdef USE_INHIBIT_TIME
  UNSIGNED16 inhibit_time;       // Inhibit timer in ms (0 if COS not used)
  UNSIGNED16 inhibit_timestamp;  // If inhibit timer is used, this is the 
                                 // timestamp for the next transmission
  UNSIGNED8 inhibit_status;      // 0: Inhibit timer not started or expired
                                 // 1: Inhibit timer started
                                 // 2: Transmit msg waiting for expiration of inhibit
#endif
  UNSIGNED8 offset;              // Offest to application data in process image
} TPDO_CONFIG;

// This structure holds all the RPDO configuration data for one RPDO
typedef struct 
{
  UNSIGNED16 CANID;              // Message Identifier 
  UNSIGNED8 len;                 // Data length (0-8) 
  UNSIGNED8 offset;              // Pointer to destination of data 
} RPDO_CONFIG;


/**************************************************************************
GLOBAL FUNCTIONS
**************************************************************************/

/**************************************************************************
DOES:    Initializes the MicroCANopen stack
         It must be called from within MCOUSER_ResetApplication
RETURNS: nothing
**************************************************************************/
void MCO_Init 
  (
  UNSIGNED16 Baudrate,    // CAN baudrate in kbit(1000,800,500,250,125,50,25 or 10)
  UNSIGNED8 Node_ID,      // CANopen node ID (1-126)
  UNSIGNED16 Heartbeat    // Heartbeat time in ms (0 for none)
  );

/**************************************************************************
DOES:    This function initializes a transmit PDO. Once initialized, the 
         MicroCANopen stack automatically handles transmitting the PDO.
         The application can directly change the data at any time.
NOTE:    For data consistency, the application should not write to the data
         while function MCO_ProcessStack executes.
RETURNS: nothing
**************************************************************************/
void MCO_InitTPDO
  (
  UNSIGNED8 PDO_NR,       // TPDO number (1-4)
  UNSIGNED16 CAN_ID,      // CAN identifier to be used (set to 0 to use default)
  UNSIGNED16 event_tim,   // Transmitted every event_tim ms 
                          // (set to 0 if ONLY inhibit_tim should be used)
  UNSIGNED16 inhibit_tim, // Inhibit time in ms for change-of-state transmit
                          // (set to 0 if ONLY event_tim should be used)
  UNSIGNED8 len,          // Number of data bytes in TPDO
  UNSIGNED8 offset        // Offset to data location in process image
  );

/**************************************************************************
DOES:    This function initializes a receive PDO. Once initialized, the 
         MicroCANopen stack automatically updates the data at offset.
NOTE:    For data consistency, the application should not read the data
         while function MCO_ProcessStack executes.
RETURNS: nothing
**************************************************************************/
void MCO_InitRPDO
  (
  UNSIGNED8 PDO_NR,       // RPDO number (1-4)
  UNSIGNED16 CAN_ID,      // CAN identifier to be used (set to 0 to use default)
  UNSIGNED8 len,          // Number of data bytes in RPDO
  UNSIGNED8 offset        // Offset to data location in process image
  );

/**************************************************************************
DOES:    This function implements the main MicroCANopen protocol stack. 
         It must be called frequently to ensure proper operation of the
         communication stack. 
         Typically it is called from the while(1) loop in main.
RETURNS: 0 if nothing was done, 1 if a CAN message was sent or received
**************************************************************************/
UNSIGNED8 MCO_ProcessStack
  (
  void
  );

/**************************************************************************
USER CALL-BACK FUNCTIONS
These must be implemented by the application.
**************************************************************************/

/**************************************************************************
DOES:    Call-back function for reset application.
         Starts the watchdog and waits until watchdog causes a reset.
RETURNS: nothing
**************************************************************************/
void MCOUSER_ResetApplication
  (
  void
  );

/**************************************************************************
DOES:    This function both resets and initializes both the CAN interface
         and the CANopen protocol stack. It is called from within the
         CANopen protocol stack, if a NMT master message was received that
         demanded "Reset Communication".
         This function should call MCO_Init and MCO_InitTPDO/MCO_InitRPDO.
RETURNS: nothing
**************************************************************************/
void MCOUSER_ResetCommunication
  (
  void
  );

/**************************************************************************
DOES:    This function is called if a fatal error occurred. 
         Error codes of mcohwxxx.c are in the range of 0x8000 to 0x87FF.
         Error codes of mco.c are in the range of 0x8800 to 0x8FFF. 
         All other error codes may be used by the application.
RETURNS: nothing
**************************************************************************/
void MCOUSER_FatalError
  (
  UNSIGNED16 ErrCode // To debug, search source code for the ErrCode encountered
  );

#endif
