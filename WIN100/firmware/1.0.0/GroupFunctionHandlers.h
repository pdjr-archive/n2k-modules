/**********************************************************************
 * GroupFunctionHandlers.h - for PGN 128776, 128777 and 128778
 * Copyright (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */
  
#ifndef _GroupFunctionHandlers_H_
#define _GroupFunctionHandlers_H_

#include "NMEA2000_CompilerDefns.h"
#include "PGN128776.h"
#include "N2kGroupFunction.h"

#if !defined(N2K_NO_GROUP_FUNCTION_SUPPORT)

class GroupFunctionHandlerForPGN128776 : public tN2kGroupFunctionHandler {
  protected:
    virtual bool HandleCommand(const tN2kMsg &N2kMsg, uint8_t PrioritySetting, uint8_t NumberOfParameterPairs, int iDev);
  public:
    GroupFunctionHandlerForPGN128776(tNMEA2000 *_pNMEA2000, void (*updateFunction)(PGN128776_Field[])) : tN2kGroupFunctionHandler(_pNMEA2000, 128776L), updateFunction(updateFunction) {};
  private:
    void (*updateFunction)(PGN128776_Field[]);
};

#endif
#endif