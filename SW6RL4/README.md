# SW6RL4 Generic Module

The __SW6RL4 Generic Module__ is a hardware design for an NMEA 2000
module which supports six optically-isolated switch inputs, four
analogue inputs and four relay outputs.
It was developed to provide a re-usable hardware platform for N2K
modules which have fairly lavish IO requirements.

The module provides seven PCB mounted DIP switches provisionally
configured as a single MODE switch and eight one-bit ADDRESS switches
which can be used for configuring the module instance address.

The module GPIO configuration is available as simple C++ header file
```SW6RL4.h``` which firmware targetted at the hardware design can
import to bootstrap development.