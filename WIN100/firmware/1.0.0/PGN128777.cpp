/**********************************************************************
 * PGN128777.cpp (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Anchor Windlass Operating Status
 */

#include <NMEA2000StdTypes.h>
#include <N2kTypes.h>
#include "PGN128777.h"

const int PGN128777::SID 1
const int PGN128777::WindlassIdentifier 2;
const int PGN128777::RodeCounterValue 3;
const int PGN128777::WindlassLineSpeed 4;
const int PGN128777::WindlassMotionStatus 5;
const int PGN128777::RodeTypeStatus 6;
const int PGN128777::AnchorDockingStatus 7;
const int PGN128777::WindlassOperatingEvents 8;

bool PGN128777::isDirty(int index) {
  return(this->properties[index].dirty);
}

void PGN128777::setClean(int index) {
  this->properties[index].dirty = false;
}

PGN128777_GenericField PGN128777::getProperty(int index) {
  return this->properties[index].value;
}

void PGN128777::setProperty(int index, PGN128777_GenericField value) {
  switch (index) {
    case PGN128777::SID: this->setSID(value.F01); break;
    case PGN128777::WindlassIdentifier: this->setWindlassIdentifier(value.F02); break;
    case PGN128777::RodeCounterValue: this->setRodeCounterValue(value.F03); break;
    case PGN128777::WindlassLineSpeed: this->setWindlassLineSpeed(value.F04); break;
    case PGN128777::WindlassMotionStatus: this->setWindlassMotionStatus(value.F05); break;
    case PGN128777::RodeTypeStatus: this->setRodeTypeStatus(value.F06); break;
    case PGN128777::AnchorDockingStatus: this->setAnchorDockingStatus(value.F07); break;
    case PGN128777::WindlassOperatingEvents: this->setWindlassOperatingEvents(value.F08); break;
    default: break;
  }
}

uint8_t PGN128777::getSID() {
  return this->properties[PGN128777::SID].value.F01;
}

uint8_t PGN128777::getWindlassIdentifier() {
  return this->properties[PGN128777::WindlassIdentifier].value.F02;
}

double PGN128777::getRodeCounterValue() {
  return this->properties[PGN128777::RodeCounterValue].value.F03;
}

double PGN128777::getc() {
  return this->properties[PGN128777::RodeCounterValue].value.F04;
}

tN2kDD480 PGN128777::getWindlassMotionStatus() {
  return this->properties[PGN128777::WindlassMotionStatus].value.F05;
}

tN2kDD481 PGN128777::getRodeTypeStatus() {
  return this->properties[PGN128777::RodeTypeStatus].value.F06;
}

tN2kDD482 PGN128777::getAnchorDockingStatus() {
  return this->properties[PGN128777::AnchorDockingStatus].value.F07;
}

tN2kDD483 PGN128777::getWindlassOperatingEvents() {
  return this->properties[PGN128777::WindlassOperatingEvents].value.F08;
}
    
void PGN128777::setSID(uint8_t SID) {
  this->properties[PGN128777::SID].dirty = true;
  this->properties[PGN128777::SID].value.F01 = SID;
}

void PGN128777::setWindlassIdentifier(uint8_t value) {
  this->properties[PGN128777::WindlassIdentifier].dirty = (this->properties[PGN128777::WindlassIdentifier].value.F02 != value);
  this->properties[PGN128777::WindlassIdentifier].value.F02 = value;
}

void PGN128777::setRodeCounterValue(double value) {
  this->properties[PGN128777::RodeCounterValue].dirty = (this->properties[PGN128777::RodeCounterValue].value.F03 != value);
  this->properties[PGN128777::RodeCounterValue].value.F03 = value;
}

void PGN128777::setWindlassLineSpeed(double value) {
  this->properties[PGN128777::WindlassLineSpeed].dirty = (this->properties[PGN128777::WindlassLineSpeed].value.F04 != value);
  this->properties[PGN128777::WindlassLineSpeed].value.F04 = value;
}

void PGN128777::setWindlassMotionStatus(tN2kDD480 value) {
  this->properties[PGN128777::WindlassMotionStatus].dirty = (this->properties[PGN128777::WindlassMotionStatus].value.F05 != value);
  this->properties[PGN128777::WindlassMotionStatus].value.F05 = value;
}

void PGN128777::setRodeTypeStatus(tN2kDD481 value) {
  this->properties[PGN128777::RodeTypeStatus].dirty = (this->properties[PGN128777::RodeTypeStatus].value.F06 != value);
  this->properties[PGN128777::RodeTypeStatus].value.F06 = value;
}

void PGN128777::setAnchorDockingStatus(tN2kDD482 value) {
  this->properties[PGN128777::AnchorDockingStatus].dirty = (this->properties[PGN128777::AnchorDockingStatus].value.F07 != value);
  this->properties[PGN128777::AnchorDockingStatus].value.F07 = value;
}

void PGN128777::setWindlassOperatingEvents(tN2kDD483 value) {
  this->properties[PGN128777::WindlassOperatingEvents].dirty = (this->properties[PGN128777::WindlassOperatingEvents].value.F08 != value);
  this->properties[PGN128777::WindlassOperatingEvents].value.F08 = value;
}
