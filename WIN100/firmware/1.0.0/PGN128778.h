/**********************************************************************
 * PGN128778.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Anchor Windlass Monitoring Status
 */

#ifndef _PGN128778_H_
#define _PGN128778_H_

#define PGN128778_FieldCount 7
#define PGN128778_StaticUpdateInterval 5000
#define PGN128778_DynamicUpdateInterval 500

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128778_GenericField {
  uint8_t F01;      // SID
  uint8_t F02;      // WindlassIdentifier
  tN2kDD477 F03;    // WindlassMonitoringEvents
  double F04;       // ControllerVoltage
  double F05;       // MotorCurrent
  double F06;       // TotalMotorTime
  uint9_t F07;      // NMEAReserved
};

/**********************************************************************
 * A structure that can be used to store any field value and also
 * record when that value is changed.
 */
struct PGN128778_Field {
  bool dirty;
  PGN128778_GenericField value;
};

class PGN128778 {
  public:
    static const int SID;
    static const int WindlassIdentifier;
    static const int WindlassMonitoringEvents;
    static const int ControllerVoltage;
    static const int MotorCurrent;
    static const int TotalMotorTime;
    
    PGN128778() : properties {
      { false, { 0 } },               // unused
      { false, { .F01 = 0 } },        // SID
      { false, { .F02 = 0 } },        // WindlassIdentifier
      { true,  { 0 } },               // WindlassMonitoringEvents
      { true,  { .F04 = 0.0 } },      // ControllerVoltage
      { true,  { .F05 = 0.0 } },      // MotorCurrent
      { true,  { .F06 = 0.0 } },      // TotalMotorTime
      { false, { .F07 = 0 } }         // NMEAReserved
      } {};
    
    PGN128778_GenericField getProperty(int index);
    void setProperty(int index, PGN128778_GenericField value);
    bool isDirty(int index);
    void setClean(int index);

    uint8_t getSID();
    uint8_t getWindlassIdentifier();
    tN2kDD477 getWindlassMonitoringEvents();
    double getControllerVoltage();
    double getMotorCurrent();
    double getTotalMotorTime();
    
    void setSID(uint8_t SID);
    void setWindlassIdentifier(uint8_t value);
    void setWindlassMonitoringEvents(tN2kDD477 value);
    void setControllerVoltage(double value);
    void setMotorCurrent(double value);
    void setTotalMotorTime(double value);
    
  private:
    PGN128778_Field properties[PGN128778_FieldCount + 1];
};

#endif