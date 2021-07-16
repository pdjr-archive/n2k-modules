/**********************************************************************
 * PGN128006.cpp (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */

#include <NMEA2000StdTypes.h>
#include <N2kTypes.h>
#include "PGN128006.h"

const int PGN128006::ThrusterIdentifier = 2;
const int PGN128006::ThrusterDirectionControl = 3;
const int PGN128006::PowerEnable = 4;
const int PGN128006::ThrusterRetractControl = 5;
const int PGN128006::SpeedControl = 6;
const int PGN128006::ThrusterControlEvents = 7;
const int PGN128006::CommandTimeout = 8;
const int PGN128006::AzimuthControl = 9;

bool PGN128006::isDirty(int index) {
  return(this->properties[index].dirty);
}

void PGN128006::setDirty(int index) {
  return(this->properties[index].dirty = true);
}

void PGN128006::setClean(int index) {
  this->properties[index].dirty = false;
}

PGN128006_GenericField PGN128006::getProperty(int index) {
  return this->properties[index].value;
}

void PGN128006::setProperty(int index, PGN128006_GenericField value) {
  switch (index) {
    case PGN128006::ThrusterIdentifier: this->setThrusterIdentifier(value.F02); break;
    case PGN128006::ThrusterDirectionControl: this->setThrusterDirectionControl(value.F03); break;
    case PGN128006::PowerEnable: this->setPowerEnable(value.F04); break;
    case PGN128006::ThrusterRetractControl: this->setThrusterRetractControl(value.F05); break;
    case PGN128006::SpeedControl: this->setSpeedControl(value.F06); break;
    case PGN128006::ThrusterControlEvents: this->setThrusterControlEvents(value.F07); break;
    case PGN128006::CommandTimeout: this->setCommandTimeout(value.F08); break;
    case PGN128006::AzimuthControl: this->setAzimuthControl(value.F09); break;
    default: break;
  }
}

uint8_t PGN128006::getThrusterIdentifier() {
  return this->properties[PGN128006::ThrusterIdentifier].value.F02;
}

tN2kDD473 PGN128006::getThrusterDirectionControl() {
  return this->properties[PGN128006::ThrusterDirectionControl].value.F03;
}

tN2kDD002 PGN128006::getPowerEnable() {
  return this->properties[PGN128006::PowerEnable].value.F04;
}

tN2kDD474 PGN128006::getThrusterRetractControl() {
  return this->properties[PGN128006::ThrusterRetractControl].value.F05;
}

uint8_t PGN128006::getSpeedControl() {
  return this->properties[PGN128006::SpeedControl].value.F06;
}

tN2kDD475 PGN128006::getThrusterControlEvents() {
  return this->properties[PGN128006::ThrusterControlEvents].value.F07;
}

double PGN128006::getCommandTimeout() {
  return this->properties[PGN128006::CommandTimeout].value.F08;
}

double PGN128006::getAzimuthControl() {
  return this->properties[PGN128006::AzimuthControl].value.F09;
}

void PGN128006::setThrusterIdentifier(uint8_t value) {
  this->properties[PGN128006::ThrusterIdentifier].dirty = (this->properties[PGN128006::ThrusterIdentifier].value.F02 != value);
  this->properties[PGN128006::ThrusterIdentifier].value.F02 = value;
}

void PGN128006::setThrusterDirectionControl(tN2kDD473 value) {
  this->properties[PGN128006::ThrusterDirectionControl].dirty = (this->properties[PGN128006::ThrusterDirectionControl].value.F03 != value);
  this->properties[PGN128006::ThrusterDirectionControl].value.F03 = value;
}

void PGN128006::setPowerEnable(tN2kDD002 value) {
  this->properties[PGN128006::PowerEnable].dirty = (this->properties[PGN128006::PowerEnable].value.F04 != value);
  this->properties[PGN128006::PowerEnable].value.F04 = value;
}

void PGN128006::setThrusterRetractControl(tN2kDD474 value) {
  this->properties[PGN128006::ThrusterRetractControl].dirty = (this->properties[PGN128006::ThrusterRetractControl].value.F05 != value);
  this->properties[PGN128006::ThrusterRetractControl].value.F05 = value;
}

void PGN128006::setSpeedControl(uint8_t value) {
  this->properties[PGN128006::SpeedControl].dirty = (this->properties[PGN128006::SpeedControl].value.F06 != value);
  this->properties[PGN128006::SpeedControl].value.F06 = value;
}

void PGN128006::setThrusterControlEvents(tN2kDD475 value) {
  this->properties[PGN128006::ThrusterControlEvents].dirty = true;
  this->properties[PGN128006::ThrusterControlEvents].value.F07 = value;
}

void PGN128006::setCommandTimeout(double value) {
  this->properties[PGN128006::CommandTimeout].dirty = (this->properties[PGN128006::CommandTimeout].value.F08 != value);
  this->properties[PGN128006::CommandTimeout].value.F08 = value;
}

void PGN128006::setAzimuthControl(double value) {
  this->properties[PGN128006::AzimuthControl].dirty = (this->properties[PGN128006::AzimuthControl].value.F09 != value);
  this->properties[PGN128006::AzimuthControl].value.F09 = value;
}
