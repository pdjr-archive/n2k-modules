/**********************************************************************
 * PGN128007.cpp (c) 2021 Paul Reeve <preeve@pdjr.eu>
 */

#include <Arduino.h>
#include <NMEA2000StdTypes.h>
#include <N2kTypes.h>
#include "PGN128007.h"

const int PGN128007::ThrusterIdentifier = 1;
const int PGN128007::ThrusterMotorType = 2;
const int PGN128007::MotorPowerRating = 4;
const int PGN128007::MaximumMotorTemperatureRating = 5;
const int PGN128007::MaximumRotationalSpeed = 6;

bool PGN128007::isDirty(int index) {
  return(this->properties[index].dirty);
}

void PGN128007::setClean(int index) {
  this->properties[index].dirty = false;
}

PGN128007_GenericField PGN128007::getProperty(int index) {
  return this->properties[index].value;
}

void PGN128007::setProperty(int index, PGN128007_GenericField value) {
  switch (index) {
    case PGN128007::ThrusterIdentifier: this->setThrusterIdentifier(value.F01); break;
    case PGN128007::ThrusterMotorType: this->setThrusterMotorType(value.F02); break;
    case PGN128007::MotorPowerRating: this->setMotorPowerRating(value.F04); break;
    case PGN128007::MaximumMotorTemperatureRating: this->setMaximumMotorTemperatureRating(value.F05); break;
    case PGN128007::MaximumRotationalSpeed: this->setMaximumRotationalSpeed(value.F06); break;
    default: break;
  }
}

uint8_t PGN128007::getThrusterIdentifier() {
  return this->properties[PGN128007::ThrusterIdentifier].value.F01;
}

tN2kDD487 PGN128007::getThrusterMotorType() {
  return this->properties[PGN128007::ThrusterMotorType].value.F02;
}

int PGN128007::getMotorPowerRating() {
  return this->properties[PGN128007::MotorPowerRating].value.F04;
}

double PGN128007::getMaximumMotorTemperatureRating() {
  return this->properties[PGN128007::MaximumMotorTemperatureRating].value.F05;
}

double PGN128007::getMaximumRotationalSpeed() {
  return this->properties[PGN128007::MaximumRotationalSpeed].value.F06;
}

void PGN128007::setThrusterIdentifier(unsigned char value) {
  this->properties[PGN128007::ThrusterIdentifier].value.F01 = value;
}

void PGN128007::setThrusterMotorType(tN2kDD487 value) {
  this->properties[PGN128007::ThrusterMotorType].value.F02 = value;
}

void PGN128007::setMotorPowerRating(int value) {
  this->properties[PGN128007::MotorPowerRating].value.F04 = value;
}

void PGN128007::setMaximumMotorTemperatureRating(double value) {
  this->properties[PGN128007::MaximumMotorTemperatureRating].value.F05 = value;
}

void PGN128007::setMaximumRotationalSpeed(double value) {
  this->properties[PGN128007::MaximumRotationalSpeed].value.F06 = value;
}

void PGN128007::serialDump() {
  Serial.println("PGN128007 {");
  Serial.print("  F01 (Thruster Identifier): ");
  Serial.println(this->properties[PGN128007::ThrusterIdentifier].value.F01, HEX);
  Serial.print("  F02 (Thruster Motor Type): ");
  switch (this->properties[PGN128007::ThrusterMotorType].value.F02) {
    case N2kDD487_12VDC: Serial.println("12VDC"); break;
    case N2kDD487_24VDC: Serial.println("24VDC"); break;
    case N2kDD487_48VDC: Serial.println("48VDC"); break;
    case N2kDD487_24VAC: Serial.println("24VAC"); break;
    case N2kDD487_Hydraulic: Serial.println("Hydraulic"); break;
  }
  Serial.print("  F04 (Motor Power Rating): ");
  Serial.println(this->properties[PGN128007::MotorPowerRating].value.F04);
  Serial.print("  F05 (Maximum Motor Temperature Rating): ");
  Serial.println(this->properties[PGN128007::MaximumMotorTemperatureRating].value.F05);
  Serial.print("  F06 (Maximum Rotational Speed): ");
  Serial.println(this->properties[PGN128007::MaximumRotationalSpeed].value.F06);
  Serial.println("}");
}
