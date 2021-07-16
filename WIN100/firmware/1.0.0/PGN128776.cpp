/**********************************************************************
 * PGN128776.cpp (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */

#include <NMEA2000StdTypes.h>
#include <N2kTypes.h>
#include "PGN128776.h"

const int PGN128776::SID = 1;
const int PGN128776::WindlassIdentifier = 2;
const int PGN128776::WindlassDirectionControl = 3;
const int PGN128776::AnchorDockingControl = 4;
const int PGN128776::SpeedControlType = 5;
const int PGN128776::SpeedControl = 7;
const int PGN128776::PowerEnable = 8;
const int PGN128776::MechanicalLock = 9;
const int PGN128776::DeckAndAnchorWash = 10;
const int PGN128776::AnchorLight = 11;
const int PGN128776::CommandTimeout = 12;
const int PGN128776::WindlassControlEvents = 13;

bool PGN128776::isDirty(int index) {
  return(this->properties[index].dirty);
}

void PGN128776::setClean(int index) {
  this->properties[index].dirty = false;
}

PGN128776_GenericField PGN128776::getProperty(int index) {
  return this->properties[index].value;
}

void PGN128776::setProperty(int index, PGN128776_GenericField value) {
  switch (index) {
    case PGN128776::SID: this->setSID(value.F01); break;
    case PGN128776::WindlassIdentifier: this->setWindlassIdentifier(value.F02); break;
    case PGN128776::WindlassDirectionControl: this->setWindlassDirectionControl(value.F03); break;
    case PGN128776::AnchorDockingControl: this->setAnchorDockingControl(value.F04); break;
    case PGN128776::SpeedControlType: this->setSpeedControlType(value.F05); break;
    case PGN128776::SpeedControl: this->setSpeedControl(value.F07); break;
    case PGN128776::PowerEnable: this->setPowerEnable(value.F08); break;
    case PGN128776::MechanicalLock: this->setMechanicalLock(value.F09); break;
    case PGN128776::DeckAndAnchorWash: this->setDeckAndAnchorWash(value.F10); break;
    case PGN128776::AnchorLight: setAnchorLight(value.F11); break;
    case PGN128776::CommandTimeout: setCommandTimeout(value.F12); break;
    case PGN128776::WindlassControlEvents: setWindlassControlEvents(value.F13); break;
    default: break;
  }
}
    
uint8_t PGN128776::getSID();
  return this->properties[PGN128776::SID].value.F01;
}

uint8_t PGN128776::getWindlassIdentifier() {
  return this->properties[PGN128776::SID].value.F02;
}

tN2kDD484 PGN128776::getWindlassDirectionControl() {
  return this->properties[PGN128776::SID].value.F03;
}

tN2kDD002 PGN128776::getAnchorDockingControl() {
  return this->properties[PGN128776::SID].value.F04;
}

tN2kDD488 PGN128776::getSpeedControlType() {
  return this->properties[PGN128776::SID].value.F05;
}

uint8_t PGN128776::getSpeedControl() {
  return this->properties[PGN128776::SID].value.F07;
}

tN2kDD002 PGN128776::getPowerEnable() {
  return this->properties[PGN128776::SID].value.F08;
}

tN2kDD002 PGN128776::getMechanicalLock() {
  return this->properties[PGN128776::SID].value.F09;
}

tN2kDD002 PGN128776::getDeckAndAnchorWash() {
  return this->properties[PGN128776::SID].value.F10;
}

tN2kDD002 PGN128776::getAnchorLight() {
  return this->properties[PGN128776::SID].value.F11;
}

double PGN128776::getCommandTimeout() {
  return this->properties[PGN128776::SID].value.F12;
}

tN2kDD478 PGN128776::getWindlassControlEvents() {
  return this->properties[PGN128776::SID].value.F13;
}

void PGN128776::setSID(uint8_t SID) {
  this->properties[PGN128776::SID].dirty = true;
  this->properties[PGN128776::SID].value.F01 = SID;
}

void PGN128776::setWindlassIdentifier(uint8_t value) {
  this->properties[PGN128776::WindlassIdentifier].dirty = true;
  this->properties[PGN128776::WindlassIdentifier].value.F02 = value;
}

void PGN128776::setWindlassDirectionControl(tN2kDD484 value) {
  this->properties[PGN128776::WindlassDirectionControl].dirty = (this->properties[PGN128776::WindlassDirectionControl].value.F03 == value);
  this->properties[PGN128776::WindlassDirectionControl].value.F03 = value;
}

void PGN128776::setAnchorDockingControl(tN2kDD002 value) {
  this->properties[PGN128776::AnchorDockingControl].dirty = (this->properties[PGN128776::AnchorDockingControl].value.F04 == value);
  this->properties[PGN128776::AnchorDockingControl].value.F04 = value;
}

void PGN128776::setSpeedControlType(tN2kDD488 value) {
  this->properties[PGN128776::SpeedControlType].dirty = (this->properties[PGN128776::SpeedControlType].value.F05 == value);
  this->properties[PGN128776::SpeedControlType].value.F05 = value;
}

void PGN128776::setSpeedControl(uint8_t value) {
  this->properties[PGN128776::SpeedControl].dirty = (this->properties[PGN128776::SpeedControl].value.F07 == value);
  this->properties[PGN128776::SpeedControl].value.F07 = value;
}

void PGN128776::setPowerEnable(tN2kDD002 value) {
  this->properties[PGN128776::PowerEnable].dirty = (this->properties[PGN128776::PowerEnable].value.F08 == value);
  this->properties[PGN128776::PowerEnable].value.F08 = value;
}

void PGN128776::setMechanicalLock(tN2kDD002 value) {
  this->properties[PGN128776::MechanicalLock].dirty = (this->properties[PGN128776::MechanicalLock].value.F09 == value);
  this->properties[PGN128776::MechanicalLock].value.F09 = value;
}

void PGN128776::setDeckAndAnchorWash(tN2kDD002 value) {
  this->properties[PGN128776::DeckAndAnchorWash].dirty = (this->properties[PGN128776::DeckAndAnchorWash].value.F10 == value);
  this->properties[PGN128776::DeckAndAnchorWash].value.F10 = value;
}

void PGN128776::setAnchorLight(tN2kDD002 value) {
  this->properties[PGN128776::AnchorLight].dirty = (this->properties[PGN128776::AnchorLight].value.F11 == value);
  this->properties[PGN128776::AnchorLight].value.F11 = value;
}

void PGN128776::setCommandTimeout(double value) {
  this->properties[PGN128776::CommandTimeout].dirty = (this->properties[PGN128776::CommandTimeout].value.F12 == value);
  this->properties[PGN128776::CommandTimeout].value.F12 = value;
}

void PGN128776::setWindlassControlEvents(tN2kDD478 value) {
  this->properties[PGN128776::WindlassControlEvents].dirty = (this->properties[PGN128776::WindlassControlEvents].value.F13 == value);
  this->properties[PGN128776::WindlassControlEvents].value.F13 = value;
}
