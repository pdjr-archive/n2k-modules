/**********************************************************************
 * PGN128006.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */

#ifndef _PGN128006_H_
#define _PGN128006_H_

#define PGN128006_FieldCount 9
#define PGN128006_StaticUpdateInterval 5000
#define PGN128006_DynamicUpdateInterval 500
#define PGN128006_CommandTransmitInterval = 250
#define PGN128006_DefaultCommandTimeoutInterval 0.35

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128006_GenericField {
  uint8_t F01;
  uint8_t F02;
  tN2kDD473 F03;
  tN2kDD002 F04;
  tN2kDD474 F05;
  uint8_t F06;
  tN2kDD475 F07;
  double F08;
  double F09;
};

/**********************************************************************
 * A structure that can be used to store any field value and also
 * record when that value is changed.
 */
struct PGN128006_Field {
  bool dirty;
  PGN128006_GenericField value;
};

class PGN128006 {
  public:
    static const int ThrusterIdentifier;
    static const int ThrusterDirectionControl;
    static const int PowerEnable;
    static const int ThrusterRetractControl;
    static const int SpeedControl;
    static const int ThrusterControlEvents;
    static const int CommandTimeout;
    static const int AzimuthControl;
    
    PGN128006() : properties {
      { false, { (uint8_t) 0 } }, // Field 0 - unused
      { false, { (uint8_t) 0 } }, // Field 1 - unused (SID)
      { false, { .F02 = (uint8_t) 255 } }, // Field 2 - ThrusterIdentifier
      { true,  { .F03 = N2kDD473_OFF } }, // Field 3 - ThrusterDirectionControl
      { true,  { .F04 = N2kDD002_Off } }, // Field 4 - PowerEnable
      { true,  { .F05 = N2kDD474_OFF } }, // Field 5 - ThrusterRetractControl
      { true,  { .F06 = (uint8_t) 0 } }, // Field 6 - SpeedControl
      { true,  { .F02 = (uint8_t) 0 } }, // Field 7 - ThrusterControlEvents
      { true,  { .F08 = (double) PGN128006_DefaultCommandTimeoutInterval } }, // Field 8 - CommandTimeout
      { true,  { .F09 = (double) 0 } } // Field 9 - AzimuthControl
    } {};
    
    PGN128006_GenericField getProperty(int index);
    void setProperty(int index, PGN128006_GenericField value);
    bool isDirty(int index);
    void setDirty(int index);
    void setClean(int index);

    uint8_t getThrusterIdentifier();
    tN2kDD473 getThrusterDirectionControl();
    tN2kDD002 getPowerEnable();
    tN2kDD474 getThrusterRetractControl();
    uint8_t getSpeedControl();
    tN2kDD475 getThrusterControlEvents();
    double getCommandTimeout();
    double getAzimuthControl();
    void setThrusterIdentifier(uint8_t value);
    void setThrusterDirectionControl(tN2kDD473 value);
    void setPowerEnable(tN2kDD002 value);
    void setThrusterRetractControl(tN2kDD474 value);
    void setSpeedControl(uint8_t value);
    void setThrusterControlEvents(tN2kDD475 value);
    void setCommandTimeout(double value);
    void setAzimuthControl(double value);
  
  private:
    PGN128006_Field properties[PGN128006_FieldCount + 1];
};

#endif