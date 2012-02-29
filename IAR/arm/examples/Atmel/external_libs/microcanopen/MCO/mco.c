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

#include "mco.h"
#include "mcohw.h"
#include <string.h>

#if USE_MICROLSS == 1
#include "mlss.h"
#endif

/**************************************************************************
GLOBAL VARIABLES
***************************************************************************/ 

// this structure holds all node specific configuration
MCO_CONFIG MEM_FAR gMCOConfig;

#if NR_OF_TPDOS > 0
// this structure holds all the TPDO configuration data for up to 4 TPDOs
TPDO_CONFIG MEM_FAR gTPDOConfig[NR_OF_TPDOS];
#endif

// this is the next TPDO to be checked in MCO_ProcessStack
UNSIGNED8 MEM_FAR gTPDONr = NR_OF_TPDOS;

#if NR_OF_RPDOS > 0
// this structure holds all the RPDO configuration data for up to 4 RPDOs
RPDO_CONFIG MEM_FAR gRPDOConfig[NR_OF_RPDOS];
#endif

// this structure holds the current receive message
CAN_MSG MEM_FAR gRxCAN;

// this structure holds the CAN message for SDO responses or aborts
CAN_MSG MEM_FAR gTxSDO;

// process image from user_xxxx.c
extern UNSIGNED8 MEM_NEAR gProcImg[];

// table with SDO Responses for read requests to OD - defined in user_xxx.c
extern UNSIGNED8 MEM_CONST SDOResponseTable[];


/**************************************************************************
LOCAL FUNCTIONS
***************************************************************************/

// SDO Abort Messages
#define SDO_ABORT_UNSUPPORTED     0x06010000UL
#define SDO_ABORT_NOT_EXISTS      0x06020000UL
#define SDO_ABORT_READONLY        0x06010002UL
#define SDO_ABORT_TYPEMISMATCH    0x06070010UL
#define SDO_ABORT_UNKNOWN_COMMAND 0x05040001UL
#define SDO_ABORT_UNKNOWNSUB      0x06090011UL

/**************************************************************************
DOES:    Search the SDO Response table for a specifc index and subindex.
RETURNS: 255 if not found, otherwise the number of the record found
         (staring at zero)
**************************************************************************/
UNSIGNED8 MCO_Search_OD
  (
  UNSIGNED16 index,   // Index of OD entry searched
  UNSIGNED8 subindex // Subindex of OD entry searched 
  )
{
  UNSIGNED8 i;
  UNSIGNED8 i_hi, hi;
  UNSIGNED8 i_lo, lo;
  UNSIGNED8 const *p;
  UNSIGNED8 const *r;

  i = 0;
  i_hi = (UNSIGNED8) (index >> 8);
  i_lo = (UNSIGNED8) index;
  r = &(SDOResponseTable[0]);
  while (i < 255)
  {
    p = r;
    // set r to next record in table
    r += 8;
    // skip command byte
    p++;
    lo = *p;
    p++;
    hi = *p;
    // if index in table is 0xFFFF, then this is the end of the table
    if ((lo == 0xFF) && (hi == 0xFF))
    {
      return 255;
    }
    if (lo == i_lo)
    { 
      if (hi == i_hi)
      { 
        p++;
        // entry found?
        if (*p == subindex)
        {
          return i;
        }
      }
    }
    i++;
  }

  // not found
  return 255;
}


/**************************************************************************
DOES:    Generates an SDO Abort Response
RETURNS: nothing
**************************************************************************/
void MCO_Send_SDO_Abort
  (
  UNSIGNED32 ErrorCode  // 4 byte SDO abort error code
  )
{
  UNSIGNED8 i;

  // construct message data
  gTxSDO.BUF[0] = 0x80;
  for (i=0;i<4;i++)
  {
    gTxSDO.BUF[4+i] = ErrorCode;
    ErrorCode >>= 8;
  }

  // transmit message
  if (!MCOHW_PushMessage(&gTxSDO))
  {
    // failed to transmit
    MCOUSER_FatalError(0x8801);
  }
}

/**************************************************************************
DOES:    Handle an incoimg SDO request.
RETURNS: returns 1 if SDO access success, returns 0 if SDO abort generated
**************************************************************************/
UNSIGNED8 MCO_Handle_SDO_Request 
  (
  UNSIGNED8 *pData  // pointer to 8 data bytes with SDO data
  )
{
  // command byte of SDO request
  UNSIGNED8 cmd;
  // index of SDO request
  UNSIGNED16 index;
  // subindex of SDO request
  UNSIGNED8 subindex;
  // search result of Search_OD
  UNSIGNED8 found;

  // init variables
  // upper 3 bits are the command
  cmd = *pData & 0xE0;
  // get high byte of index
  index = pData[2];
  // add low byte of index
  index = (index << 8) + pData[1];
  // subindex
  subindex = pData[3];

  // Copy Multiplexor into response
  // index low
  gTxSDO.BUF[1] = pData[1];
  // index high
  gTxSDO.BUF[2] = pData[2];
  // subindex
  gTxSDO.BUF[3] = pData[3];

  // is it a read or write command?
  if ((cmd == 0x40) || (cmd == 0x20)) 
  {

    // Error register
    if ((index == 0x1001) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 1 byte of data
        gTxSDO.BUF[0] = 0x4F;
        gTxSDO.BUF[4] = gMCOConfig.error_register;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(0x8802);
        }
        return 1;
      }
      // write command
      MCO_Send_SDO_Abort(SDO_ABORT_READONLY);
      return 0;
    }

    // RPDO COB-ID
    if ((index >= 0x1400) && (index < 0x1400+NR_OF_RPDOS) && (subindex == 0x01))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 4 bytes of data
        gTxSDO.BUF[0] = 0x43;
        gTxSDO.BUF[4] = (UNSIGNED8) gRPDOConfig[index&0x0003].CANID;
        gTxSDO.BUF[5] = (UNSIGNED8) (gRPDOConfig[index&0x0003].CANID >> 8);
        gTxSDO.BUF[6] = 0;
        gTxSDO.BUF[7] = 0;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(0x8802);
        }
        return 1;
      }
      // write command
      MCO_Send_SDO_Abort(SDO_ABORT_READONLY);
      return 0;
    }

    // TPDO COB-ID
    if ((index >= 0x1800) && (index < 0x1800+NR_OF_RPDOS) && (subindex == 0x01))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 4 bytes of data
        gTxSDO.BUF[0] = 0x43;
        gTxSDO.BUF[4] = (UNSIGNED8) gTPDOConfig[index&0x0003].CAN.ID;
        gTxSDO.BUF[5] = (UNSIGNED8) (gTPDOConfig[index&0x0003].CAN.ID >> 8);
        gTxSDO.BUF[6] = 0;
        gTxSDO.BUF[7] = 0;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(0x8802);
        }
        return 1;
      }
      // write command
      MCO_Send_SDO_Abort(SDO_ABORT_READONLY);
      return 0;
    }

#ifdef P101804_SERIAL_NUMBER
    // Serial Number
    if ((index == 0x1018) && (subindex == 0x04))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 4 bytes of data
        gTxSDO.BUF[0] = 0x43;
        gTxSDO.BUF[4] = gProcImg[P101804_SERIAL_NUMBER];
        gTxSDO.BUF[5] = gProcImg[P101804_SERIAL_NUMBER+1];
        gTxSDO.BUF[6] = gProcImg[P101804_SERIAL_NUMBER+2];
        gTxSDO.BUF[7] = gProcImg[P101804_SERIAL_NUMBER+3];
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(0x8802);
        }
        return 1;
      }
      // write command
      MCO_Send_SDO_Abort(SDO_ABORT_READONLY);
      return 0;
    }
#endif

#ifdef DYNAMIC_HEARTBEAT
    // hard coding of dynamic read/write accesses
    // access to [1017,00] - heartbeat time
    if ((index == 0x1017) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 2 bytes of data
        gTxSDO.BUF[0] = 0x4B;
        gTxSDO.BUF[4] = (UNSIGNED8) gMCOConfig.heartbeat_time;
        gTxSDO.BUF[5] = (UNSIGNED8) (gMCOConfig.heartbeat_time >> 8);
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(0x8802);
        }
        return 1;
      }
      // expedited write command with 2 bytes of data
      if (*pData == 0x2B)
      {
        gMCOConfig.heartbeat_time = pData[5];
        gMCOConfig.heartbeat_time = (gMCOConfig.heartbeat_time << 8) + pData[4];
        // write response
        gTxSDO.BUF[0] = 0x60;
        // Needed to pass conformance test: clear unused bytes
        gTxSDO.BUF[4] = 0;
        gTxSDO.BUF[5] = 0;
        gTxSDO.BUF[6] = 0;
        gTxSDO.BUF[7] = 0;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(0x8802);
        }
        return 1;
      }
      MCO_Send_SDO_Abort(SDO_ABORT_UNSUPPORTED);
      return 0;
    }
#endif // DYNAMIC HEARTBEAT

#ifdef REBOOT_FLAG_ADR
    // Supporting the ESAcademy CANopen Bootloader
    if ((index == 0x1F50) && (subindex == 0x01))
    {
      // expedited write command with 4 bytes of data
      if (*pData == (1<<5) + 3)
      {
        if ( ((pData[4]) == 'B') && ((pData[5]) == 'O') && 
             ((pData[6]) == 'O') && ((pData[7]) == 'T') 
           )
        { // Set signal to bootloader
          REBOOT_FLAG = REBOOT_BOOTLOAD_VAL;
          // write response
          gTxSDO.BUF[0] = 0x60;
          // Needed to pass conformance test: clear unused bytes
          gTxSDO.BUF[4] = 0;
          gTxSDO.BUF[5] = 0;
          gTxSDO.BUF[6] = 0;
          gTxSDO.BUF[7] = 0;
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(0x8802);
          }
          return 1;
        }
      }
      MCO_Send_SDO_Abort(SDO_ABORT_UNSUPPORTED);
      return 0;
    }
#endif // REBOOT_FLAG_ADR

    // search const table 
    found = MCO_Search_OD(index,subindex);
    // entry found?
    if (found < 255)
    {
      // read command?
      if (cmd == 0x40)
      {
        memcpy(&gTxSDO.BUF[0],&SDOResponseTable[(found*8)],8);
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(0x8802);
        }
        return 1;
      }
      // write command
      MCO_Send_SDO_Abort(SDO_ABORT_READONLY);
      return 0;
    }

    // Requested OD entry not found
    if (subindex == 0)
    {
      MCO_Send_SDO_Abort(SDO_ABORT_NOT_EXISTS);
    }
    else
    {
      MCO_Send_SDO_Abort(SDO_ABORT_UNKNOWNSUB);
    }
    return 0;
  }
  // ignore abort received - all other produce an error
  if (cmd != 0x80)
  {
    MCO_Send_SDO_Abort(SDO_ABORT_UNKNOWN_COMMAND);
    return 0;
  }
  return 1;
}


#if NR_OF_TPDOS > 0
/**************************************************************************
DOES:    Called when going into the operational mode.
         Prepares all TPDOs for operational.
RETURNS: nothing
**************************************************************************/
void MCO_Prepare_TPDOs 
  (
    void
  )
{
UNSIGNED8 i;

  i = 0;
  // prepare all TPDOs for transmission
  while (i < NR_OF_TPDOS)
  {
    // this TPDO is used
    if (gTPDOConfig[i].CAN.ID != 0)
    {
      // Copy current process data
      memcpy(&gTPDOConfig[i].CAN.BUF[0],&(gProcImg[gTPDOConfig[i].offset]),gTPDOConfig[i].CAN.LEN);
#ifdef USE_EVENT_TIME
      // Reset event timer for immediate transmission
      gTPDOConfig[i].event_timestamp = MCOHW_GetTime() - 2;
#endif
#ifdef USE_INHIBIT_TIME
      gTPDOConfig[i].inhibit_status = 2; // Mark as ready for transmission
      // Reset inhibit timer for immediate transmission
      gTPDOConfig[i].inhibit_timestamp = MCOHW_GetTime() - 2;
#endif
    }
  i++;
  }
  // ensure that MCO_ProcessStack starts with TPDO1
  gTPDONr = NR_OF_TPDOS;
}

/**************************************************************************
DOES:    Called when a TPDO needs to be transmitted
RETURNS: nothing
**************************************************************************/
void MCO_TransmitPDO 
  (
  UNSIGNED8 PDONr  // TPDO number to transmit
  )
{
#ifdef USE_INHIBIT_TIME
  // new inhibit timer started
  gTPDOConfig[PDONr].inhibit_status = 1;
  gTPDOConfig[PDONr].inhibit_timestamp = MCOHW_GetTime() + gTPDOConfig[PDONr].inhibit_time;
#endif
#ifdef USE_EVENT_TIME
  gTPDOConfig[gTPDONr].event_timestamp = MCOHW_GetTime() + gTPDOConfig[gTPDONr].event_time; 
#endif
  if (!MCOHW_PushMessage(&gTPDOConfig[PDONr].CAN))
  {
    MCOUSER_FatalError(0x8801);
  }
}
#endif // NR_OF_TPDOS > 0

/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/ 

/**************************************************************************
DOES:    Initializes the MicroCANopen stack
         It must be called from within MCOUSER_ResetApplication
RETURNS: nothing
**************************************************************************/
void MCO_Init 
  (
  UNSIGNED16 Baudrate,  // CAN baudrate in kbit (1000,800,500,250,125,50,25 or 10)
  UNSIGNED8 Node_ID,   // CANopen node ID (1-126)
  UNSIGNED16 Heartbeat  // Heartbeat time in ms (0 for none)
  )
{
  UNSIGNED8 i;

  // Init the global variables
  gMCOConfig.Node_ID = Node_ID;
  gMCOConfig.error_code = 0;
  gMCOConfig.Baudrate = Baudrate;
  gMCOConfig.heartbeat_time = Heartbeat;
  gMCOConfig.heartbeat_msg.ID = 0x700+Node_ID;
  gMCOConfig.heartbeat_msg.LEN = 1;
  // current NMT state of this node = bootup
  gMCOConfig.heartbeat_msg.BUF[0] = 0;
  gMCOConfig.error_register = 0;
 
  // Init SDO Response/Abort message
  gTxSDO.ID = 0x580+gMCOConfig.Node_ID;
  gTxSDO.LEN = 8;
   
  // init the CAN interface
  if (!MCOHW_Init(Baudrate))
  {
    MCOUSER_FatalError(0x8802);
  }

#if USE_MICROLSS == 1
  // Set receive filter for LSS master message 
  if (!MCOHW_SetCANFilter(LSS_MASTER_ID))
  {
    MCOUSER_FatalError(0x88F4);
  }
  if (gMCOConfig.Node_ID == 0)
  { // force LSS mode
    gMCOConfig.heartbeat_msg.BUF[0] = NMTSTATE_LSS;
    return; // abort initialization here
  }
#endif

#if NR_OF_TPDOS > 0
  i = 0;
  // init TPDOs
  while (i < NR_OF_TPDOS)
  {
    gTPDOConfig[i].CAN.ID = 0;
    i++;
  }
#endif
#if NR_OF_RPDOS > 0
  i = 0;
  // init RPDOs
  while (i < NR_OF_RPDOS)
  {
    gRPDOConfig[i].CANID = 0;
    i++;
  }
#endif

  // for nmt master message
  if (!MCOHW_SetCANFilter(0))
  {
    MCOUSER_FatalError(0x8803);
  }
  // for SDO requests
  if (!MCOHW_SetCANFilter(0x600+Node_ID))
  {
    MCOUSER_FatalError(0x8803);
  }

  // signal to MCO_ProcessStack: we just initialized
  gTPDONr = 0xFF;
}  

#if NR_OF_RPDOS > 0
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
  UNSIGNED16 CAN_ID,       // CAN identifier to be used (set to 0 to use default)
  UNSIGNED8 len,          // Number of data bytes in RPDO
  UNSIGNED8 offset        // Offset to data location in process image
  )
{

#ifdef CHECK_PARAMETERS
  // check PDO range and check node id range 1 - 127
  if (((PDO_NR < 1)             || (PDO_NR > NR_OF_RPDOS))      || 
      ((gMCOConfig.Node_ID < 1) || (gMCOConfig.Node_ID > 127)))
  {
    MCOUSER_FatalError(0x8804);
  }
  // is size of process image exceeded?
  if (offset > PIMGEND)   
  { 
    MCOUSER_FatalError(0x8904);
  }
#endif
  PDO_NR--;
  gRPDOConfig[PDO_NR].len = len;
  gRPDOConfig[PDO_NR].offset = offset;
  if (CAN_ID == 0)
  {
    gRPDOConfig[PDO_NR].CANID = 0x200 + (0x100 * ((UNSIGNED16)(PDO_NR))) + gMCOConfig.Node_ID;
  }
  else
  {
    gRPDOConfig[PDO_NR].CANID = CAN_ID;
  }
  if (!MCOHW_SetCANFilter(gRPDOConfig[PDO_NR].CANID))
  {
    MCOUSER_FatalError(0x8805);
  }
}
#endif // NR_OF_RPDOS > 0


#if NR_OF_TPDOS > 0
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
  UNSIGNED8 PDO_NR,        // TPDO number (1-4)
  UNSIGNED16 CAN_ID,        // CAN identifier to be used (set to 0 to use default)
  UNSIGNED16 event_time,    // Transmitted every event_tim ms 
  UNSIGNED16 inhibit_time,  // Inhibit time in ms for change-of-state transmit
                      // (set to 0 if ONLY event_tim should be used)
  UNSIGNED8 len,           // Number of data bytes in TPDO
  UNSIGNED8 offset         // Offset to data location in process image
  )
{

#ifdef CHECK_PARAMETERS
  // check PDO range, node id, len range 1 - 8 and event time or inhibit time set
  if (((PDO_NR < 1)             || (PDO_NR > NR_OF_TPDOS))     ||
      ((gMCOConfig.Node_ID < 1) || (gMCOConfig.Node_ID > 127)) ||
      ((len < 1)                || (len > 8))                  ||
      ((event_time == 0)        && (inhibit_time == 0)))
  {
    MCOUSER_FatalError(0x8806);
  }
  // is size of process image exceeded?
  if (offset > PIMGEND)   
  { 
    MCOUSER_FatalError(0x8906);
  }
#endif
  PDO_NR--;
  if (CAN_ID == 0)
  {
    gTPDOConfig[PDO_NR].CAN.ID = 0x180 + (0x100 * ((UNSIGNED16)(PDO_NR))) + gMCOConfig.Node_ID;
  }
  else
  {
    gTPDOConfig[PDO_NR].CAN.ID = CAN_ID;
  }
  gTPDOConfig[PDO_NR].CAN.LEN = len;
  gTPDOConfig[PDO_NR].offset = offset;
#ifdef USE_EVENT_TIME
  gTPDOConfig[PDO_NR].event_time = event_time;
#endif
#ifdef USE_INHIBIT_TIME
  gTPDOConfig[PDO_NR].inhibit_time = inhibit_time;
#endif
}
#endif // NR_OF_TPDOS > 0


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
  )
{
  UNSIGNED8 i;
  UNSIGNED8 ret_val = 0;

  // check if this is right after boot-up
  // was set by MCO_Init
  if (gTPDONr == 0xFF)
  {
    // init heartbeat time
    gMCOConfig.heartbeat_timestamp = MCOHW_GetTime() + gMCOConfig.heartbeat_time;
    // send boot-up message  
    if (!MCOHW_PushMessage(&gMCOConfig.heartbeat_msg))
    {
      MCOUSER_FatalError(0x8801);
    }
#ifdef AUTOSTART
    // going into operational state
    gMCOConfig.heartbeat_msg.BUF[0] = NMTSTATE_OP;
#if NR_OF_TPDOS > 0
    MCO_Prepare_TPDOs();
#endif
#else
    // going into pre-operational state
    gMCOConfig.heartbeat_msg.BUF[0] = NMTSTATE_PREOP;
#endif
    // return value to default
    gTPDONr = NR_OF_TPDOS;
    return 1;
  }
 
  // work on next incoming messages
  // if message received
  if (MCOHW_PullMessage(&gRxCAN))
  {

#if USE_MICROLSS == 1
    if (gRxCAN.ID == LSS_MASTER_ID)
    {
      LSS_HandleMsg(gRxCAN.LEN,&(gRxCAN.BUF[0]));
      return 1;
    }
#endif

    // is it an NMT master message?
    if (gRxCAN.ID == 0)
    {
      // nmt message is for this node or all nodes
      if ((gRxCAN.BUF[1] == gMCOConfig.Node_ID) || (gRxCAN.BUF[1] == 0))
      {
        switch (gRxCAN.BUF[0])
        {
          // start node
          case NMTMSG_OP:
            gMCOConfig.heartbeat_msg.BUF[0] = NMTSTATE_OP;
#if NR_OF_TPDOS > 0          
            MCO_Prepare_TPDOs();
#endif
            break;

          // stop node
          case NMTMSG_STOP:
            gMCOConfig.heartbeat_msg.BUF[0] = NMTSTATE_STOP;
            break;

          // enter pre-operational
          case NMTMSG_PREOP:
            gMCOConfig.heartbeat_msg.BUF[0] = NMTSTATE_PREOP;
            break;
   
          // node reset
          case NMTMSG_RESETAPP:
            MCOUSER_ResetApplication();
            break;

          // node reset communication
          case NMTMSG_RESETCOM:
            MCOUSER_ResetCommunication();
            break;

          // unknown command
          default:
            break;
        }

        return 1;
      } // NMT message addressed to this node
    } // NMT master message received
    
    // if node is not stopped...
    if (gMCOConfig.heartbeat_msg.BUF[0] != NMTSTATE_STOP)
    {
      // is the message an SDO request message for us?
      if (gRxCAN.ID == gMCOConfig.Node_ID+0x600)
      {
        // handle SDO request - return value not used in this version
        i = MCO_Handle_SDO_Request(&gRxCAN.BUF[0]);
        return 1;
      }
    }

#if NR_OF_RPDOS > 0
    // is the node operational?
    if (gMCOConfig.heartbeat_msg.BUF[0] == NMTSTATE_OP)
    {
      i = 0;
      // loop through RPDOs
      while (i < NR_OF_RPDOS)
      {
        // is this one of our RPDOs?
        if (gRxCAN.ID == gRPDOConfig[i].CANID)
        {
          // copy data from RPDO to process image
          memcpy(&(gProcImg[gRPDOConfig[i].offset]),&(gRxCAN.BUF[0]),gRPDOConfig[i].len);
          // exit the loop
          i = NR_OF_RPDOS;
          ret_val = 1;
        }
        i++;
      } // for all RPDOs
    } // node is operational
#endif // NR_OF_RPDOS > 0

  } // Message received

#if USE_MICROLSS == 1
  if (LSS_DoLSS()) // work on LSS
  {
    return 1; // Do nothing else when in LSS mode
  }
#endif

#if NR_OF_TPDOS > 0
  // is the node operational?
  if (gMCOConfig.heartbeat_msg.BUF[0] == NMTSTATE_OP)
  {
    // check next TPDO for transmission
    gTPDONr++;
    if (gTPDONr >= NR_OF_TPDOS)
    {
      gTPDONr = 0;
    }
    // is the TPDO 'gTPDONr' in use?
    if (gTPDOConfig[gTPDONr].CAN.ID != 0)
    {
#ifdef USE_EVENT_TIME
      // does TPDO use event timer and event timer is expired? if so we need to transmit now
      if ((gTPDOConfig[gTPDONr].event_time != 0) && 
          (MCOHW_IsTimeExpired(gTPDOConfig[gTPDONr].event_timestamp)) )
      {
        // get data from process image and transmit
        memcpy(&(gTPDOConfig[gTPDONr].CAN.BUF[0]),&(gProcImg[gTPDOConfig[gTPDONr].offset]),gTPDOConfig[gTPDONr].CAN.LEN);
        MCO_TransmitPDO(gTPDONr);
        return 1;
      }
#endif // USE_EVENT_TIME
#ifdef USE_INHIBIT_TIME
      // does the TPDO use an inhibit time? - COS transmission
      if (gTPDOConfig[gTPDONr].inhibit_time != 0)
      {
        // is the inihibit timer currently running?
        if (gTPDOConfig[gTPDONr].inhibit_status > 0)
        {
          // has the inhibit time expired?
          if (MCOHW_IsTimeExpired(gTPDOConfig[gTPDONr].inhibit_timestamp))
          {
            // is there a new transmit message already waiting?
            if (gTPDOConfig[gTPDONr].inhibit_status == 2)
            { 
              // transmit now
              MCO_TransmitPDO(gTPDONr);
              return 1;
            }
            // no new message waiting, but timer expired
            else 
            {
              gTPDOConfig[gTPDONr].inhibit_status = 0;
            }
          }
        }
        // is inhibit status 0 or 1?
        if (gTPDOConfig[gTPDONr].inhibit_status < 2)
        {
          // has application data changed?
          if ((memcmp(&gTPDOConfig[gTPDONr].CAN.BUF[0],&(gProcImg[gTPDOConfig[gTPDONr].offset]),gTPDOConfig[gTPDONr].CAN.LEN) != 0))
          {
            // Copy application data
            memcpy(&gTPDOConfig[gTPDONr].CAN.BUF[0],&(gProcImg[gTPDOConfig[gTPDONr].offset]),gTPDOConfig[gTPDONr].CAN.LEN);
            // has inhibit time expired?
            if (gTPDOConfig[gTPDONr].inhibit_status == 0)
            {
              // transmit now
              MCO_TransmitPDO(gTPDONr);
              return 1;
            }
            // inhibit status is 1
            else
            {
              // wait for inhibit time to expire 
              gTPDOConfig[gTPDONr].inhibit_status = 2;
            }
          }
        }
      } // Inhibit Time != 0
#endif // USE_INHIBIT_TIME
    } // PDO active (CAN_ID != 0)  
  } // if node is operational
#endif // NR_OF_TPDOS > 0
  
  // do we produce a heartbeat?
  if ((gMCOConfig.heartbeat_time != 0) 
#if USE_MICROLSS
       && (gMCOConfig.heartbeat_msg.BUF[0] != NMTSTATE_LSS)
       && (gMCOConfig.heartbeat_msg.BUF[0] != NMTSTATE_STOP)
#endif
     )
  {
    // has heartbeat time passed?
    if (MCOHW_IsTimeExpired(gMCOConfig.heartbeat_timestamp))
    {
      // transmit heartbeat message
      if (!MCOHW_PushMessage(&gMCOConfig.heartbeat_msg))
      {
        MCOUSER_FatalError(0x8801);
      }
      // get new heartbeat time for next transmission
      gMCOConfig.heartbeat_timestamp = MCOHW_GetTime() + gMCOConfig.heartbeat_time;
      ret_val = 1;
    }
  }
  return ret_val;
}

