/**********************************************************************
 * Windlass.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Utility class representing a windlass.
 */

#ifndef _WINDLASS_H_
#define _WINDLASS_H_

#include <PGN128776.h>
#include <PGN128777.h>
#include <PGN128778.h>

class Windlass {
  public:
    unsigned char SOURCE_ADDRESS;
    unsigned long SOURCE_ADDRESS_UPDATE_TIMESTAMP;
    unsigned long OPERATING_TIMEOUT;
    unsigned long START_TIME;
    PGN128776 PGN128776v;
    PGN128777 PGN128777v;
    PGN128778 PGN128778v;
        
    Windlass() : { 0x22, 0UL, 0UL, 0UL, PGN128776(), PGN128777(), PGN128778() } {};
    
    void setWindlassIdentifier(unit8_t identifier);
    unit8_t getWindlassIdentifier();

    int saveToEEEPROM(int address);
    void loadFromEEPROM(int address);
};

#endif