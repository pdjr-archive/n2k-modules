# SW6RL4 Generic Module

The __SW6RL4__ is a hardware design for an NMEA 2000 module which
provides generalised support for devices which require multiple
switch and sensor inputs and relay outputs.

The design offers:

* six optically-isolated 12/24VDC switch inputs;
* four analogue inputs suitable for voltage measurement (the module
  provides 5V reference and ground connections for each analogue
  input);
* four SPST relay outputs rated at 5A DC;
* LED output for power / status indication;
* 9 DIP switch configuration inputs suitable for setting a module
  instance address, etc.

The module GPIO configuration is available as simple C++ header file
[SW6RL4.h](SW6RL4.h) which firmware applications can import to
bootstrap development.