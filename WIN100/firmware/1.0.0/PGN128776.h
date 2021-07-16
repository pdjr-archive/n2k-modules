/**********************************************************************
 * PGN128776.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Anchor Windlass Control Status
 */

#ifndef _PGN128776_H_
#define _PGN128776_H_

#define PGN128776_FieldCount 14
#define PGN128776_StaticUpdateInterval 5000
#define PGN128776_DynamicUpdateInterval 500
#define PGN128776_DefaultCommandTimeoutInterval 0.35

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128776_GenericField {
  uint8_t F01;        // F01 SID
  uint8_t F02;        // F02 WindlassIdentifier
  tN2kDD484 F03;      // F03 WindlassDirectionControl
  tN2kDD002 F04;      // F04 AnchorDockingControl
  tN2kDD488 F05;      // F05 SpeedControlType
  uint8_t F06;        // F06 NMEAReserved
  tN2kDD489 F07;      // F07 SpeedControl
  tN2kDD002 F08;      // F08 PowerEnable
  tN2kDD002 F09;      // F09 MechanicalLock
  tN2kDD002 F10;      // F10 DeckAndAnchorWash
  tN2kDD002 F11;      // F11 AnchorLight
  double F12;         // F12 CommandTimeout
  tN2kDD478 F13;      // F13 WindlassControlEvents
  uint8_t F14         // F14 NMEAReserved
};

/**********************************************************************
 * A structure that can be used to store any field value and also
 * record when that value is changed.
 */
struct PGN128776_Field {
  bool dirty;
  PGN128776_GenericField value;
};

class PGN128776 {
  public:
    static const int SID;
    static const int WindlassIdentifier;
    static const int WindlassDirectionControl;
    static const int AnchorDockingControl;
    static const int SpeedControlType;
    static const int SpeedControl;
    static const int PowerEnable;
    static const int MechanicalLock;
    static const int DeckAndAnchorWash;
    static const int AnchorLight;
    static const int CommandTimeout;
    static const int WindlassControlEvents;
    
    PGN128776() : properties {
      { false, { (uint8_t) 0 } },                       // Unused
      { false, { .F01 = 0 } },                          // SID
      { false, { .F02 = 0 } },                          // WindlassIdentifier
      { true,  { .F03 = N2kDD484_Off } },               // WindlessDirectionControl
      { true,  { .F04 = N2kDD002_Unknown } },           // AnchorDockingControl
      { true,  { .F05 = N2kDD488_DataNotAvailable } },  // SpeedControlType
      { false, { .F06 = 0 } },                          // NMEAReserved
      { true,  { .F07 = 0 } },                          // SpeedControl
      { true,  { .F08 = N2kDD002_Unknown } },           // PowerEnable
      { true,  { .F09 = N2kDD002_Unknown } },           // MechanicalLock
      { true,  { .F10 = N2kDD002_Unknown } },           // DeckAndAnchorWash
      { true,  { .F11 = N2kDD002_Unknown } },           // AnchorLight
      { true,  { .F12 = 0.0 } },                        // CommandTimeout
      { true,  { .F13 = 0 } },                          // WindlassControlEvents
      { false, { .F14 = 0}} }                           // NMEAReserved
    } {};
    
    PGN128776_GenericField getProperty(int index);
    void setProperty(int index, PGN128776_GenericField value);
    bool isDirty(int index);
    void setClean(int index);

    uint8_t getSID();
    uint8_t getWindlassIdentifier();
    tN2kDD484 getWindlassDirectionControl();
    tN2kDD002 getAnchorDockingControl();
    tN2kDD488 getSpeedControlType();
    uint8_t getSpeedControl();
    tN2kDD002 getPowerEnable();
    tN2kDD002 getMechanicalLock();
    tN2kDD002 getDeckAndAnchorWash();
    tN2kDD002 getAnchorLight();
    double getCommandTimeout();
    tN2kDD478 getWindlassControlEvents();

    void setSID(uint8_t SID);
    void setWindlassIdentifier(uint8_t value);
    void setWindlassDirectionControl(tN2kDD484 value);
    void setAnchorDockingControl(tN2kDD002 value);
    void setSpeedControlType(tN2kDD488 value);
    void setSpeedControl(uint8_t value);
    void setPowerEnable(tN2kDD002 value);
    void setMechanicalLock(tN2kDD002 value);
    void setDeckAndAnchorWash(tN2kDD002 value);
    void setAnchorLight(tN2kDD002 value);
    void setCommandTimeout(double value);
    void setWindlassControlEvents(tN2kDD478 value);
    
  private:
    PGN128776_Field properties[PGN128776_FieldCount + 1];
};

#endif