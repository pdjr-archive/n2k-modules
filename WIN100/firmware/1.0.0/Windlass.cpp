/**********************************************************************
 * Windlass.cpp (c) 2021 Paul Reeve <preeve@pdjr.eu>
 * Utility class representing a windlass.
 */

#include <EEPROM.h>
#include <N2kTypes.h>
#include <Windlass.h>

void Windlass::setWindlassIdentifier(uint8_t identifier) {
  this->PGN128776v.setWindlassIdentifier(identifier);
  this->PGN128777v.setWindlassIdentifier(identifier);
  this->PGN128778v.setWindlassIdentifier(identifier);
}

uint8_t Windlass::getWindlassIdentifier() {
  return(this->PGN128776v.getWindlassIdentifier());
}

int Windlass::saveToEEEPROM(int address) {
  int storageAddress = address;
  EEPROM.put(storageAddress, this->SOURCE_ADDRESS); storageAddress += sizeof(this->SOURCE_ADDRESS);
  EEPROM.put(storageAddress, this->SOURCE_ADDRESS_UPDATE_TIMESTAMP); storageAddress += sizeof(SOURCE_ADDRESS_UPDATE_TIMESTAMP);
  EEPROM.put(storageAddress, this->START_TIME); storageAddress += sizeof(this->START_TIME);
  EEPROM.put(storageAddress, this->GPIO_UP_SWITCH); storageAddress += sizeof(this->GPIO_UP_SWITCH);
  EEPROM.put(storageAddress, this->GPIO_DN_SWITCH); storageAddress += sizeof(this->GPIO_DN_SWITCH);
  EEPROM.put(storageAddress, this->PGN128776v); storageAddress += sizeof(this->PGN128776v);
  EEPROM.put(storageAddress, this->PGN128777v); storageAddress += sizeof(this->PGN128777v);
  EEPROM.put(storageAddress, this->PGN128778v); storageAddress += sizeof(this->PGN128778v);
  return(storageAddress - address);
}

void Windlass::loadFromEEPROM(int address) {
  int storageAddress = address;
  EEPROM.get(storageAddress, this->SOURCE_ADDRESS); storageAddress += sizeof(this->SOURCE_ADDRESS);
  EEPROM.get(storageAddress, this->SOURCE_ADDRESS_UPDATE_TIMESTAMP); storageAddress += sizeof(SOURCE_ADDRESS_UPDATE_TIMESTAMP);
  EEPROM.get(storageAddress, this->START_TIME); storageAddress += sizeof(this->START_TIME);
  EEPROM.get(storageAddress, this->GPIO_UP_SWITCH); storageAddress += sizeof(this->GPIO_UP_SWITCH);
  EEPROM.get(storageAddress, this->GPIO_DN_SWITCH); storageAddress += sizeof(this->GPIO_DN_SWITCH);
  EEPROM.get(storageAddress, this->PGN128776v); storageAddress += sizeof(this->PGN128776v);
  EEPROM.get(storageAddress, this->PGN128777v); storageAddress += sizeof(this->PGN128777v);
  EEPROM.get(storageAddress, this->PGN128778v); storageAddress += sizeof(this->PGN128778v);
  return;
}
  
