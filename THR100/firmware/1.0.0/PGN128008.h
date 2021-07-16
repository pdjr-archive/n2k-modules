/**********************************************************************
 * PGN128008.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */

#ifndef _PGN128008_H_
#define _PGN128008_H_

#define PGN128008_FieldCount 6
#define PGN128008_StaticUpdateInterval 5000
#define PGN128008_DynamicUpdateInterval 500

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128008_GenericField {
  uint8_t F01;
  uint8_t F02;
  tN2kDD471 F03;
  uint8_t F04;
  double F05;
  unsigned long F06;
};

/**********************************************************************
 * A structure that can be used to store any field value and also
 * record when that value is changed.
 */
struct PGN128008_Field {
  bool dirty;
  PGN128008_GenericField value;
};

class PGN128008 {
  public:
    static const int SID;
    static const int ThrusterIdentifier;
    static const int ThrusterMotorEvents;
    static const int MotorCurrent;
    static const int MotorTemperature;
    static const int TotalMotorOperatingTime;

    PGN128008() : properties {
      { false, { 0 } }, // Field 0 - unused
      { false, { .F01 = 0 } }, // Field 1 - unused (SID)
      { true, { .F02 = 255 } }, // Field 2 - ThrusterIdentifier
      { true, { .F01 = 0 } }, // Field 3 - ThrusterMotorEvents
      { true, { .F04 = 0 } }, // Field 4 - MotorCurrent
      { true, { .F05 = 0.0 } }, // Field 5 - MotorTemperature
      { true, { .F06 = 0UL } } // Field 6 - TotalMotorOperatingTime  
    } {};

    PGN128008_GenericField getProperty(int index);
    void setProperty(int index, PGN128008_GenericField value);
    bool isDirty(int index);
    void setClean(int index);

    uint8_t getSID();
    uint8_t getThrusterIdentifier();
    tN2kDD471 getThrusterMotorEvents();
    uint8_t getMotorCurrent();
    double getMotorTemperature();
    unsigned long getTotalMotorOperatingTime();
    void setSID(uint8_t value);
    void setThrusterIdentifier(uint8_t value);
    void setThrusterMotorEvents(tN2kDD471 value);
    void setMotorCurrent(uint8_t value);
    void setMotorTemperature(double value);
    void setTotalMotorOperatingTime(unsigned long seconds);
    void bumpTotalMotorOperatingTime(unsigned long seconds);

    void serialDump();

  private:
    PGN128008_Field properties[PGN128008_FieldCount + 1];
};

#endif