/**********************************************************************
 * PGN128778.cpp (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Anchor Windlass Monitoring Status
 */

#include <NMEA2000StdTypes.h>
#include <N2kTypes.h>
#include "PGN128778.h"

const int PGN128778::SID = 1;
const int PGN128778::WindlassIdentifier = 2;
const int PGN128778::WindlassMonitoringEvents = 3;
const int PGN128778::ControllerVoltage = 4;
const int PGN128778::MotorCurrent = 5;
const int PGN128778::TotalMotorTime = 6;
    
bool PGN128778::isDirty(int index) {
  return(this->properties[index].dirty);
}

void PGN128778::setClean(int index) {
  this->properties[index].dirty = false;
}

PGN128778_GenericField PGN128778::getProperty(int index) {
  return this->properties[index].value;
}

void PGN128778::setProperty(int index, PGN128778_GenericField value) {
  switch (index) {
    case PGN128778::SID: this->setSID(value.F01); break;
    case PGN128778::WindlassIdentifier: this->setWindlassIdentifier(value.F02); break;
    case PGN128778::WindlassMonitoringEvents: this->setWindlassMonitoringEvents(value.F03); break;
    case PGN128778::ControllerVoltage: this->setControllerVoltage(value.F04); break;
    case PGN128778::MotorCurrent: this->setMotorCurrent(value.F05); break;
    case PGN128778::TotalMotorTime: this->setTotalMotorTime(value.F06); break;
    default: break;
  }
}
    
uint8_t getSID() {
  return this->properties[PGN128778::SID].value.F01;
}

uint8_t getWindlassIdentifier() {
  return this->properties[PGN128778::WindlassIdentifier].value.F02;
}

tN2kDD477 getWindlassMonitoringEvents() {
  return this->properties[PGN128778::WindlassMonitoringEvents].value.F03;
}

double getControllerVoltage() {
  return this->properties[PGN128778::ControllerVoltage].value.F04;
}

double getMotorCurrent() {
  return this->properties[PGN128778::MotorCurrent].value.F05;
}

double getTotalMotorTime() {
  return this->properties[PGN128778::TotalMotorTime].value.F06;
}

void setSID(uint8_t SID) {
  this->properties[PGN128778::SID].dirty = (this->properties[PGN128778::SID].value.F01 != SID);
  this->properties[PGN128778::SID].value.F01 = SID;
}

void setWindlassIdentifier(uint8_t value) {
  this->properties[PGN128778::WindlassIdentifier].dirty = (this->properties[PGN128778::WindlassIdentifier].value.F02 != value);
  this->properties[PGN128778::WindlassIdentifier].value.F02 = value;
}

void setWindlassMonitoringEvents(tN2kDD477 value) {
  this->properties[PGN128778::WindlassMonitoringEvents].dirty = (this->properties[PGN128778::WindlassMonitoringEvents].value.F03 != value);
  this->properties[PGN128778::WindlassMonitoringEvents].value.F03 = value;
}

void setControllerVoltage(double value) {
  this->properties[PGN128778::ControllerVoltage].dirty = (this->properties[PGN128778::ControllerVoltage].value.F04 != value);
  this->properties[PGN128778::ControllerVoltage].value.F04 = value;
}

void setMotorCurrent(double value) {
  this->properties[PGN128778::MotorCurrent].dirty = (this->properties[PGN128778::MotorCurrent].value.F05 != value);
  this->properties[PGN128778::MotorCurrent].value.F05 = value;
}

void setTotalMotorTime(double value) {
  this->properties[PGN128778::TotalMotorTime].dirty = (this->properties[PGN128778::TotalMotorTime].value.F06 != value);
  this->properties[PGN128778::TotalMotorTime].value.F06 = value;
}


