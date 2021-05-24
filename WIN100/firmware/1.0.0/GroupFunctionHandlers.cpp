/**********************************************************************
 * GroupFunctionHandlers.cpp
 * Copyright (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * 
 * This file implements a number of classes providing group function
 * support for PGNs 128776, 128777 and 128778.
 * 
 * GroupFunctionHandlerForPGN128776 implements a Command function
 * handler which allows a remote to update any field.
 * 
 * GroupFunctionHandlerForPGN128777 implements a Request function
 * handler which allows a remote to request transmission of the
 * eponymous PGN. Note that requests for individual field values are
 * not honoured - any request results in transmission of a complete
 * PGN
 */ 

#include <string.h>
#include <NMEA2000StdTypes.h>
#include "GroupFunctionHandlers.h"
#include "NMEA2000.h"

#if !defined(N2K_NO_GROUP_FUNCTION_SUPPORT)

///////////////////////////////////////////////////////////////////////
// START OF HANDLERS FOR PGN128006
///////////////////////////////////////////////////////////////////////

bool GroupFunctionHandlerForPGN128776::HandleCommand(const tN2kMsg &N2kMsg, uint8_t PrioritySetting, uint8_t NumberOfParameterPairs, int iDev) {
  int i;
  int Index;
  uint8_t field;
  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = N2kgfTPec_Acknowledge;
  tN2kGroupFunctionParameterErrorCode PARec;
  tN2kMsg N2kRMsg;
  int canUpdate = true;
  PGN128776_Field fields[] = { {false,0},{false,0},{false,0},{false,0},{false,0},{false,0},{false,0},{false,0},{false,0},{false,0},{false,0},{false,0},{false,0} };

 	if (PrioritySetting != 0x08 || PrioritySetting != 0x0f || PrioritySetting != 0x09) pec = N2kgfTPec_TransmitIntervalOrPriorityNotSupported;

  SetStartAcknowledge(
    N2kRMsg,
    N2kMsg.Source,
    128776L,
    N2kgfPGNec_Acknowledge,  // What we actually should response as PGN error, if we have invalid field?
    pec,
    NumberOfParameterPairs
  );

  StartParseCommandPairParameters(N2kMsg, Index);
  for (i = 0; i < NumberOfParameterPairs; i++) {
    field = N2kMsg.GetByte(Index);
    PARec = N2kgfpec_Acknowledge;
    switch (field) {
      case 2:
        fields[field].dirty = true;
        fields[field].value.F02 = N2kMsg.GetByte(Index);
        break;
      case 3:
        switch(N2kMsg.GetByte(Index)) {
          case 0: fields[field].dirty = true; fields[field].value.F03 = N2kDD484_Off; break;
          case 1: fields[field].dirty = true; fields[field].value.F03 = N2kDD484_Down; break;
          case 2: fields[field].dirty = true; fields[field].value.F03 = N2kDD484_Up; break;
          case 3: fields[field].dirty = true; fields[field].value.F03 = N2kDD484_Reserved; break;
          default: PARec = N2kgfpec_RequestOrCommandParameterOutOfRange; canUpdate = false; break;
        }
        break;
      case 4:
        fields[field].dirty = true;
        fields[field].value.F04 = N2kMsg.GetByte(Index);
        break;
      case 5:
        switch(N2kMsg.GetByte(Index)) {
          case 0: fields[field].dirty = true; fields[field].value.F05 = N2kDD488_SingleSpeed; break;
          case 1: fields[field].dirty = true; fields[field].value.F05 = N2kDD488_DualSpeed; break;
          case 2: fields[field].dirty = true; fields[field].value.F05 = N2kDD488_ProportionalSpeed; break;
          case 3: fields[field].dirty = true; fields[field].value.F05 = N2kDD488_DataNotAvailable; break;
          default: PARec = N2kgfpec_RequestOrCommandParameterOutOfRange; canUpdate = false; break;
        }
        break;
      case 6:
        switch(N2kMsg.GetByte(Index)) {
          case 0: fields[field].dirty = true; fields[field].value.F06 = N2kDD002_Off; break;
          case 1: fields[field].dirty = true; fields[field].value.F06 = N2kDD002_On; break;
          case 2: fields[field].dirty = true; fields[field].value.F06 = N2kDD002_Error; break;
          case 3: fields[field].dirty = true; fields[field].value.F06 = N2kDD002_Unavailable; break;
          default: PARec = N2kgfpec_RequestOrCommandParameterOutOfRange; canUpdate = false; break;
        }
        break;
      case 7:
        switch(N2kMsg.GetByte(Index)) {
          case 0: fields[field].dirty = true; fields[field].value.F07 = N2kDD002_Off; break;
          case 1: fields[field].dirty = true; fields[field].value.F07 = N2kDD002_On; break;
          case 2: fields[field].dirty = true; fields[field].value.F07 = N2kDD002_Error; break;
          case 3: fields[field].dirty = true; fields[field].value.F07 = N2kDD002_Unavailable; break;
          default: PARec = N2kgfpec_RequestOrCommandParameterOutOfRange; canUpdate = false; break;
        }
        break;
      case 8:
        switch(N2kMsg.GetByte(Index)) {
          case 0: fields[field].dirty = true; fields[field].value.F08 = N2kDD002_Off; break;
          case 1: fields[field].dirty = true; fields[field].value.F08 = N2kDD002_On; break;
          case 2: fields[field].dirty = true; fields[field].value.F08 = N2kDD002_Error; break;
          case 3: fields[field].dirty = true; fields[field].value.F08 = N2kDD002_Unavailable; break;
          default: PARec = N2kgfpec_RequestOrCommandParameterOutOfRange; canUpdate = false; break;
        }
        break;
      case 9:
        switch(N2kMsg.GetByte(Index)) {
          case 0: fields[field].dirty = true; fields[field].value.F09 = N2kDD002_Off; break;
          case 1: fields[field].dirty = true; fields[field].value.F09 = N2kDD002_On; break;
          case 2: fields[field].dirty = true; fields[field].value.F09 = N2kDD002_Error; break;
          case 3: fields[field].dirty = true; fields[field].value.F09 = N2kDD002_Unavailable; break;
          default: PARec = N2kgfpec_RequestOrCommandParameterOutOfRange; canUpdate = false; break;
        }
        break;
      case 10:
        switch(N2kMsg.GetByte(Index)) {
          case 0: fields[field].dirty = true; fields[field].value.F10 = N2kDD002_Off; break;
          case 1: fields[field].dirty = true; fields[field].value.F10 = N2kDD002_On; break;
          case 2: fields[field].dirty = true; fields[field].value.F10 = N2kDD002_Error; break;
          case 3: fields[field].dirty = true; fields[field].value.F10 = N2kDD002_Unavailable; break;
          default: PARec = N2kgfpec_RequestOrCommandParameterOutOfRange; canUpdate = false; break;
        }
        break;
      case 11:
        fields[field].dirty = true;
        fields[field].value.F11 = N2kMsg.Get1ByteDouble(0.005, Index);
        break;
      case 12:
        fields[field].dirty = true;
        fields[field].value.F12 = N2kMsg.GetByte(Index);
        break;
      default:
        PARec = N2kgfpec_InvalidRequestOrCommandParameterField;
    }
    AddAcknowledgeParameter(N2kRMsg, i, PARec);
  }
  pNMEA2000->SendMsg(N2kRMsg, iDev);

  if (canUpdate) this->updateFunction(fields);

  return true;
}

///////////////////////////////////////////////////////////////////////
// END OF HANDLERS FOR PGN128776
///////////////////////////////////////////////////////////////////////

#endif