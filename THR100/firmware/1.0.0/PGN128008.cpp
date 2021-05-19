/**********************************************************************
 * PGN128008.cpp (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */

#include <Arduino.h>
#include <NMEA2000StdTypes.h>
#include <N2kTypes.h>
#include "PGN128008.h"

const int PGN128008::SID = 1;
const int PGN128008::ThrusterIdentifier = 2;
const int PGN128008::ThrusterMotorEvents = 3;
const int PGN128008::MotorCurrent = 4;
const int PGN128008::MotorTemperature = 5;
const int PGN128008::TotalMotorOperatingTime = 6;

bool PGN128008::isDirty(int index) {
  return(this->properties[index].dirty);
}

void PGN128008::setClean(int index) {
  this->properties[index].dirty = false;
}

PGN128008_GenericField PGN128008::getProperty(int index) {
  return this->properties[index].value;
}

void PGN128008::setProperty(int index, PGN128008_GenericField value) {
  switch (index) {
    case PGN128008::SID: this->setSID(value.F01); break;
    case PGN128008::ThrusterIdentifier: this->setThrusterIdentifier(value.F02); break;
    case PGN128008::ThrusterMotorEvents: this->setThrusterMotorEvents(value.F03); break;
    case PGN128008::MotorCurrent: this->setMotorCurrent(value.F04); break;
    case PGN128008::MotorTemperature: this->setMotorTemperature(value.F05); break;
    case PGN128008::TotalMotorOperatingTime: this->setTotalMotorOperatingTime(value.F06); break;
    default: break;
  }
}

uint8_t PGN128008::getSID() {
  return this->properties[PGN128008::SID].value.F01;
}

uint8_t PGN128008::getThrusterIdentifier() {
  return this->properties[PGN128008::ThrusterIdentifier].value.F02;
}

tN2kDD471 PGN128008::getThrusterMotorEvents() {
  return this->properties[PGN128008::ThrusterMotorEvents].value.F03;
}

uint8_t PGN128008::getMotorCurrent() {
  return this->properties[PGN128008::MotorCurrent].value.F04;
}

double PGN128008::getMotorTemperature() {
  return this->properties[PGN128008::MotorTemperature].value.F05;
}

unsigned long PGN128008::getTotalMotorOperatingTime() {
  return this->properties[PGN128008::TotalMotorOperatingTime].value.F06;
}

void PGN128008::setSID(uint8_t value) {
  this->properties[PGN128008::SID].dirty = true;
  this->properties[PGN128008::SID].value.F01 = value;
}

void PGN128008::setThrusterIdentifier(uint8_t value) {
  this->properties[PGN128008::ThrusterIdentifier].dirty = (this->properties[PGN128008::ThrusterIdentifier].value.F02 != value);
  this->properties[PGN128008::ThrusterIdentifier].value.F02 = value;
}

void PGN128008::setThrusterMotorEvents(tN2kDD471 value) {
  this->properties[PGN128008::ThrusterMotorEvents].dirty = true;
  this->properties[PGN128008::ThrusterMotorEvents].value.F03 = value;
}

void PGN128008::setMotorCurrent(uint8_t value) {
  this->properties[PGN128008::MotorCurrent].dirty = (this->properties[PGN128008::MotorCurrent].value.F04 != value);
  this->properties[PGN128008::MotorCurrent].value.F04 = value;
}

void PGN128008::setMotorTemperature(double value) {
  this->properties[PGN128008::MotorTemperature].dirty = (this->properties[PGN128008::MotorTemperature].value.F05 != value);
  this->properties[PGN128008::MotorTemperature].value.F05 = value;
}

void PGN128008::setTotalMotorOperatingTime(unsigned long seconds) {
  this->properties[PGN128008::TotalMotorOperatingTime].dirty = (this->properties[PGN128008::TotalMotorOperatingTime].value.F06 != seconds);
  this->properties[PGN128008::TotalMotorOperatingTime].value.F06 = seconds;
}

void PGN128008::bumpTotalMotorOperatingTime(unsigned long seconds) {
  this->properties[PGN128008::TotalMotorOperatingTime].dirty = true;
  this->properties[PGN128008::TotalMotorOperatingTime].value.F06 += seconds;
}

void PGN128008::serialDump() {
  Serial.println("PGN128008 {");
  Serial.print("  F1 (SID): ");
  Serial.println(this->properties[PGN128008::SID].value.F01);
  Serial.print("  F2 (Thruster Identifier): ");
  Serial.println(this->properties[PGN128008::ThrusterIdentifier].value.F02);
  Serial.print("  F3 (Thruster Motor Events): ");
  if (this->properties[PGN128008::ThrusterMotorEvents].value.F03.Event.MotorOverTemperatureCutout) Serial.print("Motor over-temperature cutout ");
  if (this->properties[PGN128008::ThrusterMotorEvents].value.F03.Event.MotorOverCurrentCutout) Serial.print("Motor over-current cutout ");
  if (this->properties[PGN128008::ThrusterMotorEvents].value.F03.Event.LowOilLevelWarning) Serial.print("Low oil-level warning ");
  if (this->properties[PGN128008::ThrusterMotorEvents].value.F03.Event.OilOverTemperatureWarning) Serial.print("Oil over-temperature warning ");
  if (this->properties[PGN128008::ThrusterMotorEvents].value.F03.Event.ControllerUnderVoltageCutout) Serial.print("Controller under-voltage cutout ");
  Serial.println();
  Serial.print("  F4 (Motor Current): ");
  Serial.println(this->properties[PGN128008::MotorCurrent].value.F04);
  Serial.print("  F5 (Motor Temperature): ");
  Serial.println(this->properties[PGN128008::MotorTemperature].value.F05);
  Serial.print("  F6 (Total Motor Operating Time): ");
  Serial.println(this->properties[PGN128008::TotalMotorOperatingTime].value.F06);
  Serial.println("}");
}
