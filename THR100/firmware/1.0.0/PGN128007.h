/**********************************************************************
 * PGN128007.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */

#ifndef _PGN128007_H_
#define _PGN128007_H_

#define PGN128007_FieldCount 6
#define PGN128007_StaticUpdateInterval 0
#define PGN128007_DynamicUpdateInterval 0

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128007_GenericField {
  unsigned char F01;
  tN2kDD487 F02;
  int F04;
  double F05;
  double F06;
};

/**********************************************************************
 * A structure that can be used to store any field value and also
 * record when that value is changed.
 */
struct PGN128007_Field {
  bool dirty;
  PGN128007_GenericField value;
};

class PGN128007 {
  public:
    static const int ThrusterIdentifier;
    static const int ThrusterMotorType;
    static const int MotorPowerRating;
    static const int MaximumMotorTemperatureRating;
    static const int MaximumRotationalSpeed;

    PGN128007() : properties {
      { false, { (uint8_t) 0 } }, // Field 0 - unused
      { true, { .F01 = 255 } }, // Field 1 - ThrusterIdentifier
      { true, { .F02 = N2kDD487_Hydraulic } }, // Field 2 - ThrusterMotorType
      { true, { .F04 = 0 } }, // Field 4 - MotorPowerRating
      { true, { .F05 = 0.0 } }, // Field 5 - MaximumMotorTemperatureRating
      { true, { .F06 = 0.0 } } // Field 6 - MaximumRotationalSpeed
    } {};

    PGN128007_GenericField getProperty(int index);
    void setProperty(int index, PGN128007_GenericField value);
    bool isDirty(int index);
    void setClean(int index);

    uint8_t getThrusterIdentifier();
    tN2kDD487 getThrusterMotorType();
    int getMotorPowerRating();
    double getMaximumMotorTemperatureRating();
    double getMaximumRotationalSpeed();
    void setThrusterIdentifier(uint8_t value);
    void setThrusterMotorType(tN2kDD487 value);
    void setMotorPowerRating(int value);
    void setMaximumMotorTemperatureRating(double value);
    void setMaximumRotationalSpeed(double value);

    void serialDump();

  private:
    PGN128007_Field properties[PGN128007_FieldCount + 1];
};

#endif