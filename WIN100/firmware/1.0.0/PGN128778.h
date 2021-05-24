/**********************************************************************
 * PGN128778.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Anchor Windlass Monitoring Status
 */

#ifndef _PGN128778_H_
#define _PGN128778_H_

#define PGN128778_FieldCount 6
#define PGN128778_StaticUpdateInterval 5000
#define PGN128778_DynamicUpdateInterval 500

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128778_GenericField {
  uint8_t F01;      // SID
  uint8_t F02;      // WindlassIdentifier
  double F03;       // TotalMotorTime
  double F04;       // ControllerVoltage
  double F05;       // MotorCurrent
  const F06;        // WindlassMonitoringEvents
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
    static const int TotalMotorTime;
    static const int ControllerVoltage;
    static const int MotorCurrent;
    static const int WindlassMonitoringEvents;
    
    PGN128778() : properties {
      { false, { 0 } },               // Field 0 - unused
      { false, { .F01 = 0 } },        // Field 1 - SID
      { false, { .F02 = 0 } },        // Field 2 - WindlassIdentifier
      { true,  { .F03 = 0.0 } },      // Field 3 - TotalMotorTime
      { true,  { .F04 = 0.0 } },      // Field 4 - ControllerVoltage
      { true,  { .F05 = 0.0 } },      // Field 5 - MotorCurrent
      { true,  { 0 } }                // Field 6 - WindlassMonitoringEvents
    } {};
    
    PGN128778_GenericField getProperty(int index);
    void setProperty(int index, PGN128778_GenericField value);
    bool isDirty(int index);
    void setClean(int index);

    uint8_t getSID();
    uint8_t getWindlassIdentifier();
    double getTotalMotorTime();
    double getControllerVoltage();
    double getMotorCurrent();
    tN2kDD477 getWindlassMonitoringEvents();
    
    void setSID(uint8_t SID);
    void setWindlassIdentifier(uint8_t value);
    void setTotalMotorTime(double value);
    void setControllerVoltage(double value);
    void setMotorCurrent(double value);
    void setWindlassMonitoringEvents(tN2kDD477 value);
    
  private:
    PGN128778_Field properties[PGN128778_FieldCount + 1];
};

#endif