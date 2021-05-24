/**********************************************************************
 * PGN128777.h (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Anchor Windlass Operating Status
 */

#ifndef _PGN128777_H_
#define _PGN128777_H_

#define PGN128777_FieldCount 8
#define PGN128777_StaticUpdateInterval 5000
#define PGN128777_DynamicUpdateInterval 500

/**********************************************************************
 * A union that covers all possible field types in this PGN.
 */
union PGN128777_GenericField {
  uint8_t F01;    // SID
  uint8_t F02;    // WindlassIdentifier
  double F03;     // RodeCounterValue
  double F04;     // WindlassLineSpeed
  tN2kDD480 F05;  // WindlassMotionStatus
  tN2kDD481 F06;  // RodeTypeStatus
  tN2kDD482 F07;  // AnchorDockingStatus
  tN2kDD483 F08;  // WindlassOperatingEvents
};

/**********************************************************************
 * A structure that can be used to store any field value and also
 * record when that value is changed.
 */
struct PGN128777_Field {
  bool dirty;
  PGN128777_GenericField value;
};

class PGN128777 {
  public:
    static const int SID;
    static const int WindlassIdentifier;
    static const int RodeCounterValue;
    static const int WindlassLineSpeed;
    static const int WindlassMotionStatus;
    static const int RodeTypeStatus;
    static const int AnchorDockingStatus;
    static const int WindlassOperatingEvents;
    
    PGN128777() : properties {
      { false, { 0 } },                                 // Field 0 - unused
      { false, { .F01 = 0 } },                          // Field 1 - SID
      { false, { .F02 = 0 } },                          // Field 2 - WindlassIdentifier
      { true,  { .F03 = 0.0 } },                        // Field 3 - RodeCounterValue
      { true,  { .F04 = 0.0 } },                        // Field 4 - WindlassLineSpeed
      { true,  { .F05 = N2kDD480_Unavailable } },       // Field 5 - WindlassMotionStatus
      { true,  { .F06 = N2kDD481_Unavailable } },       // Field 6 - RodeTypeStatus
      { true,  { .F07 = N2kDD482_DataNotAvailable } },  // Field 7 - AnchorDockingStatus
      { true,  { 0 } }                                  // Field 8 - WindlassOperatingEvents
    } {};
    
    PGN128777_GenericField getProperty(int index);
    void setProperty(int index, PGN128777_GenericField value);
    bool isDirty(int index);
    void setClean(int index);

    uint8_t getSID();
    uint8_t getWindlassIdentifier();
    double getRodeCounterValue();
    double getWindlassLineSpeed();
    tN2kDD480 getWindlassMotionStatus();
    tN2kDD481 getRodeTypeStatus();
    tN2kDD482 getAnchorDockingStatus();
    tN2kDD483 getWindlassOperatingEvents();
    
    void setSID(uint8_t SID);
    void setWindlassIdentifier(uint8_t value);
    void setRodeCounterValue(double value);
    void setWindlassLineSpeed(double value);
    void setWindlassMotionStatus(tN2kDD480 value);
    void setRodeTypeStatus(tN2kDD481 value);
    void setAnchorDockingStatus(tN2kDD482 value);
    void setWindlassOperatingEvents(tN2kDD483 value);
    
  private:
    PGN128777_Field properties[PGN128777_FieldCount + 1];
};

#endif