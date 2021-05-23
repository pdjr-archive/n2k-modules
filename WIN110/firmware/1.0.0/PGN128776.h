/**********************************************************************
 * PGN128776.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Anchor Windlass Control Status
 */

#ifndef _PGN128776_H_
#define _PGN128776_H_

#define PGN128776_FieldCount 12
#define PGN128776_StaticUpdateInterval 5000
#define PGN128776_DynamicUpdateInterval 500
#define PGN128776_DefaultCommandTimeoutInterval 0.35

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128776_GenericField {
  uint8_t F01;
  uint8_t F02;
  tN2kDD484 F03;
  uint8_t F04;
  tN2kDD488 F05;
  tN2kDD002 F06;
  tN2kDD002 F07;
  tN2kDD002 F08;
  tN2kDD002 F09;
  tN2kDD002 F10;
  double F11;
  tN2kDD478 F12;
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
    static const int SpeedControl;
    static const int SpeedControlType;
    static const int AnchorDockingControl;
    static const int PowerEnable;
    static const int MechanicalLock;
    static const int DeckAndAnchorWash;
    static const int AnchorLight;
    static const int CommandTimeout;
    static const int WindlassControlEvents;
    
    PGN128776() : properties {
      { false, { (uint8_t) 0 } },                      // Field 0 - unused
      { false, { .F01 = 0 } },                         // Field 1 - SID
      { false, { .F02 = 0 } },                         // Field 2 - WindlassIdentifier
      { true,  { .F03 = N2kDD484_Off } },              // Field 3 - WindlessDirectionControl
      { true,  { .F04 = 0 } },                         // Field 4 - SpeedControl
      { true,  { .F05 = N2kDD488_DataNotAvailable } }, // Field 5 - SpeedControlType
      { true,  { .F06 = N2kDD002_Unknown } },          // Field 6 - AnchorDockingControl
      { true,  { .F07 = N2kDD002_Unknown } },          // Field 7 - PowerEnable
      { true,  { .F08 = N2kDD002_Unknown } },          // Field 8 - MechanicalLock
      { true,  { .F09 = N2kDD002_Unknown } },          // Field 9 - DeckAndAnchorWash
      { true,  { .F10 = N2kDD002_Unknown } },          // Field 10 - AnchorLight
      { true,  { .F11 = 0.0 } },                       // Field 11 - CommandTimeout
      { true,  { .F12 = 0 } }                          // Field 12 - WindlassControlEvents
    } {};
    
    PGN128776_GenericField getProperty(int index);
    void setProperty(int index, PGN128776_GenericField value);
    bool isDirty(int index);
    void setClean(int index);

    uint8_t getSID();
    uint8_t getWindlassIdentifier();
    tN2kDD484 getWindlassDirectionControl();
    uint8_t getSpeedControl();
    tN2kDD488 getSpeedControlType();
    tN2kDD002 getAnchorDockingControl();
    tN2kDD002 getPowerEnable();
    tN2kDD002 getMechanicalLock();
    tN2kDD002 getDeckAndAnchorWash();
    tN2kDD002 getAnchorLight();
    double getCommandTimeout();
    tN2kDD478 getWindlassControlEvents();

    void setSID(uint8_t SID);
    void setWindlassIdentifier(uint8_t value);
    void setWindlassDirectionControl(tN2kDD484 value);
    void setSpeedControl(uint8_t value);
    void setSpeedControlType(tN2kDD488 value);
    void setAnchorDockingControl(tN2kDD002 value);
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