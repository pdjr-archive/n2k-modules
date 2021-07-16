/**********************************************************************
 * THR100.cpp - THRMOD firmware version 1.0.0.
 * Copyright (c) 2021 Paul Reeve, <preeve@pdjr.eu>
 *
 * This firmware is targetted at the SW6RL4 hardware platform and
 * implements two module behaviours (SWITCH and RELAY) based on the
 * NMEA 2000 Thruster Network Messages protocol (PGNs 128006, 128007
 * and 128008). The particular behaviour expressed by the firmware is
 * determined by the state of the SW6RL4 MODE DIP switch.
 * 
 * The SWITCH behaviour allows the module to be used as a user
 * interface to physical thruster controls and indicators. The firmware
 * responds to switch and analogue inputs by transmitting control
 * messages to a remote thruster. Status messages received from the
 * remote thruster are used to operate the module's relay outputs and
 * so can be to switch UI indicators and alarms.
 * 
 * The RELAY behaviour allows the module to control a physical
 * thruster. Relay outputs are operated in response to received command
 * messages and sensor inputs are used to generate status report
 * messages which are transmitted back to the contolling device.
 * 
 * Two modules, one in SWITCH mode and one in RELAY mode, can be used
 * to control a physical thruster over the NMEA 2000 bus.
 */

#include <Arduino.h>
#include <ADC.h>
#include <EEPROM.h>
#include <NMEA2000_CAN.h>
#include <N2kTypes.h>
#include <N2kMessages.h>
#include <Debouncer.h>
#include <LedManager.h>
#include <DilSwitch.h>
#include <arraymacros.h>
#include "PGN128006.h"
#include "PGN128007.h"
#include "PGN128008.h"
#include "GroupFunctionHandlers.h"
#include <SW6RL4.h>

/**********************************************************************
 * SERIAL DEBUG
 * 
 * Define DEBUG_SERIAL to enable serial output and arrange for the
 * function debugDump() to be called from loop() every
 * DEBUG_SERIAL_INTERVAL ms. DEBUG_SERIAL_START_DELAY prevents data
 * being written to the serial port immediately after system boot.
 */

#define DEBUG_SERIAL
#define DEBUG_SERIAL_START_DELAY 4000
#define DEBUG_SERIAL_INTERVAL 1000UL

/**********************************************************************
 * MCU EEPROM (PERSISTENT) STORAGE
 * 
 * EEPROM_CONFIGURED_OPERATING_MODE. We save the last configured
 * operating mode so that we can tell (i) if the module has been
 * configured at all and (ii) if the module operating mode has been
 * changed by moving the hardware switch. Relevant to both module
 * operating modes.
 * 
 * EEPROM_SOURCE_ADDRESS. The CAN bus negotiates a storage address
 * each device and expects a device to re-use a previously negotiated
 * address if possible, so here is some storage for it. Relevant to
 * both module operating modes.
 * 
 * EEPROM_PGN128007. PGN128007 properties describe the static physical
 * and operating characteristics of a thruster. We configure this when
 * we build the module firmware and save this here. Applies only to a
 * module in relay mode.
 * 
 * EEPROM_PGN128008. PGN128008 properties describe dynamic properties
 * of a thruster. Of these, only motor run time needs to be persisted,
 * but it is convenient just to save and restore the whole PGN.
 * item.
 */

#define EEPROM_CONFIGURED_OPERATING_MODE 0x00 // 1 byte
#define EEPROM_SOURCE_ADDRESS 0x10 // 1 byte
#define EEPROM_PGN128007 0x20 // ? bytes
#define EEPROM_PGN128008 0x30 // ? bytes

/**********************************************************************
 * MCU PIN DEFINITIONS
 * 
 * GPIO pin definitions. We are using the generic SW6RL4 module, so
 * these are just aliases for that hardware's generic definitions.
 */


// PIN DEFINITIONS THAT APPLY TO SWITCH AND RELAY OPERATING MODES /////
//
#define GPIO_POWER_LED GPIO_LED_1

// SWITCH MODE PIN DEFINITIONS ////////////////////////////////////////
//
#define GPIO_STARBOARD_SWITCH GPIO_SWITCH_1
#define GPIO_PORT_SWITCH GPIO_SWITCH_2
#define GPIO_RETRACT_SWITCH GPIO_SWITCH_3
#define GPIO_SPEED_ANALOG_IN GPIO_ANALOG_1
#define GPIO_AZIMUTH_ANALOG_IN GPIO_ANALOG_2
#define GPIO_INDICATOR_PORT GPIO_RELAY_1
#define GPIO_INDICATOR_STARBOARD GPIO_RELAY_2
#define GPIO_INDICATOR_RETRACT GPIO_RELAY_3
#define GPIO_ALARM_INDICATOR_RELAY GPIO_RELAY_4

// RELAY MODE PIN DEFINITIONS /////////////////////////////////////////
//
#define GPIO_PORT_RELAY GPIO_RELAY_1
#define GPIO_STARBOARD_RELAY GPIO_RELAY_2
#define GPIO_COMMON_RELAY GPIO_RELAY_3
#define GPIO_TEMPERATURE_ANALOG GPIO_ANALOG_1
#define GPIO_PORT_SENSOR GPIO_SWITCH_1
#define GPIO_STARTBOARD_SENSOR GPIO_SWITCH 2


/**********************************************************************
 * DEVICE INFORMATION
 * 
 * Because of NMEA's closed standard, most of this is fiction. Maybe it
 * can be made better with more research. In particular, even recent
 * releases of the NMEA function and class lists found using Google
 * don't discuss anchor systems, so the proper values for CLASS and
 * FUNCTION in this application are not known.  At the moment they are
 * set to 25 (network device) and 130 (PC gateway).
 * 
 * INDUSTRY_GROUP we can be confident about (4 says maritime). However,
 * MANUFACTURER_CODE is only allocated to subscribed NMEA members and,
 * unsurprisingly, an anonymous code has not been assigned: 2046 is
 * currently unused, so we adopt that.  
 * 
 * MANUFACTURER_CODE and UNIQUE_NUMBER together must make a unique
 * value on any N2K bus and an easy way to achieve this is just to
 * bump the unique number for every software build and this is done
 * automatically by the build system.
 */
#define DEVICE_CLASS 75
#define DEVICE_FUNCTION 130
#define DEVICE_INDUSTRY_GROUP 4
#define DEVICE_MANUFACTURER_CODE 2046
#define DEVICE_UNIQUE_NUMBER 849

/**********************************************************************
 * PRODUCT INFORMATION
 * 
 * This poorly structured set of values is what NMEA expects a product
 * description to be shoe-horned into.
 */
#define PRODUCT_CERTIFICATION_LEVEL 1
#define PRODUCT_CODE 002
#define PRODUCT_FIRMWARE_VERSION "1.0.0 (May 2021)"
#define PRODUCT_LEN 1
#define PRODUCT_N2K_VERSION 2101
#define PRODUCT_SERIAL_CODE "002-849"
#define PRODUCT_TYPE "THRMOD"
#define PRODUCT_VERSION "1.0 (May 2021)"

/**********************************************************************
 * THRUSTER PROGRAMMED DEFAULTS - these will differ for each target
 * installation and should be set in external configuration.
 */
#define THRUSTER_SPEED_CONTROL 100
#define THRUSTER_AZIMUTH_CONTROL 0.0
#define THRUSTER_MOTOR_TYPE 4
#define THRUSTER_MOTOR_CURRENT 0
#define THRUSTER_MOTOR_TEMPERATURE 273
#define THRUSTER_MOTOR_POWER_RATING 8000
#define THRUSTER_MAXIMUM_MOTOR_TEMPERATURE_RATING 373.0
#define THRUSTER_MAXIMUM_ROTATIONAL_SPEED 2000
 
/**********************************************************************
 * Include the build.h header file which can be used to override any or
 * all of the above  constant definitions.
 */
#include "build.h"

#define DEFAULT_SOURCE_ADDRESS 22         // Seed value for address claim
#define DEFAULT_TOTAL_MOTOR_OPERATING_TIME 0
#define BROADCAST_SOURCE_ADDRESS 255
#define THRUSTER_SOURCE_ADDRESS_TIMEOUT 12000
#define INSTANCE_UNDEFINED 255            // Flag value
#define STARTUP_SETTLE_PERIOD 5000        // Wait this many ms before processing switch inputs
#define SWITCH_PROCESS_INTERVAL 100       // Process switch inputs evety n ms
#define LED_MANAGER_HEARTBEAT 100         // Number of ms on / off
#define LED_MANAGER_INTERVAL 10           // Number of heartbeats between repeats

#define DEFAULT_COMMAND_UPDATE_INTERVAL 250     // N2K spec says every 250ms.
#define DEFAULT_COMMAND_TIMEOUT 0.35            // In seconds.

/**********************************************************************
 * Declarations of local functions.
 */
#ifdef DEBUG_SERIAL
#endif
// CONTROL mode functions...
void processSwitchInputs();
void processAnalogControlInputs();
void transmitThrusterControl();
void PGN128006Handler(const tN2kMsg &N2kMsg);
void PGN128007Handler(const tN2kMsg &N2kMsg);
void PGN128008Handler(const tN2kMsg &N2kMsg);
void checkConnection();
bool checkEvents();

// OPERATING mode functions....
void transmitStatus();
void transmitPGN128006(unsigned char SID);
void transmitPGN128007(unsigned char SID);
void transmitPGN128008(unsigned char SID);
bool isOperating();
bool checkTimeout(unsigned long timeout);
void updatePGN128006(PGN128006_Field fields[]);
bool ISORequestHandler(unsigned long RequestedPGN, unsigned char Requester, int DeviceIndex);
// Generic functions...
void messageHandler(const tN2kMsg&);

/**********************************************************************
 * PGNs of messages we transmit. These depend on the module's operating 
 * mode, so we just initialise an empty list here and will populate it
 * in setup().
 */
unsigned long TransmitMessages[5] = { 0, 0, 0, 0, 0 };

/**********************************************************************
 * PGNs of messages we receive and their handlers.  These depend on the
 * module's operating mode, so we just initialise an empty list here
 * and will populate it in setup().
 */
typedef struct { unsigned long PGN; void (*Handler)(const tN2kMsg &N2kMsg); } tNMEA2000Handler;
tNMEA2000Handler NMEA2000Handlers[5] = { {0UL,0}, {0UL,0}, {0UL,0}, {0UL,0}, {0UL,0} };


/**********************************************************************
 * ADDRESS_SWITCH switch decoder.
 */
int ADDRESS_ENCODER_PINS[] = GPIO_ADDRESS_PINS;
DilSwitch ADDRESS_SWITCH (ADDRESS_ENCODER_PINS, ELEMENTCOUNT(ADDRESS_ENCODER_PINS));

/**********************************************************************
 * DEBOUNCER for the programme switch.
 */
int SWITCHES[DEBOUNCER_SIZE] = { GPIO_PORT_SWITCH, GPIO_STARBOARD_SWITCH, GPIO_POWER_SWITCH, GPIO_RETRACT_SWITCH, -1, -1, -1, -1 };
Debouncer DEBOUNCER (SWITCHES);

/**********************************************************************
 * ADC converter service.
 */
ADC *adc = new ADC();

/**********************************************************************
 * LED_MANAGER for all system LEDs.
 */
LedManager LED_MANAGER (LED_MANAGER_HEARTBEAT, LED_MANAGER_INTERVAL);

unsigned char THRUSTER_SOURCE_ADDRESS = BROADCAST_SOURCE_ADDRESS;
unsigned long THRUSTER_SOURCE_ADDRESS_UPDATE_TIMESTAMP = 0UL;
enum OPERATING_MODE_TYPE { SWITCH_INTERFACE, RELAY_INTERFACE } SELECTED_OPERATING_MODE = SWITCH_INTERFACE;

unsigned long PGN128006_UPDATE_INTERVAL = PGN128006_StaticUpdateInterval;
unsigned long PGN128007_UPDATE_INTERVAL = PGN128007_StaticUpdateInterval;
unsigned long PGN128008_UPDATE_INTERVAL = PGN128008_StaticUpdateInterval;
unsigned long THRUSTER_START_TIME = 0UL;

/**********************************************************************
 * These three PGNs represent the controlled thruster state. In RELAY
 * mode their properties are set in reponse to commands received and
 * sensor inputs. In SWITCH mode they are updated each time a status
 * update PGN is received from the remote thruster and are used to
 * operate indicator outputs.
 */
PGN128006 PGN128006v = PGN128006();
PGN128007 PGN128007v = PGN128007();
PGN128008 PGN128008v = PGN128008();

/**********************************************************************
 * This PGN is used in SWITCH mode to store the properties that will
 * command the remote thruster.
 */
PGN128006 PGN128006c = PGN128006();

/**********************************************************************
 * MAIN PROGRAM - setup()
 */
void setup() {
  #ifdef DEBUG_SERIAL
  Serial.begin(9600);
  delay(DEBUG_SERIAL_START_DELAY);
  #endif

  // Set the mode of all digital GPIO pins.
  int ipins[] = GPIO_DIGITAL_INPUT_PINS;
  int opins[] = GPIO_DIGITAL_OUTPUT_PINS;
  for (unsigned int i = 0 ; i < ELEMENTCOUNT(ipins); i++) pinMode(ipins[i], INPUT_PULLUP);
  for (unsigned int i = 0 ; i < ELEMENTCOUNT(opins); i++) pinMode(opins[i], OUTPUT);

  // Get the user-selected operating mode and the mode that was last configured.
  SELECTED_OPERATING_MODE = (!digitalRead(GPIO_MODE))?SWITCH_INTERFACE:RELAY_INTERFACE;
  OPERATING_MODE_TYPE CONFIGURED_OPERATING_MODE = EEPROM.read(EEPROM_CONFIGURED_OPERATING_MODE, CONFIGURED_OPERATING_MODE);
  
  // Configure the module to suit the selected operating mode. If the
  // operating mode has been changed, then this may require a bit of
  // effort...
  switch (SELECTED_OPERATING_MODE) {
    case SWITCH_INTERFACE:
      // We transmit PGN126208 and receive PGN128006, PGN128007 and
      // PGN128008.
      TransmitMessages[0] = 126208UL;
      NMEA2000Handlers[0] = { 128006UL, &PGN128006Handler };
      NMEA2000Handlers[1] = { 128007UL, &PGN128007Handler };
      NMEA2000Handlers[2] = { 128008UL, &PGN128008Handler };
      // If operating mode has changed or we have never configured
      // ourself, then set up EEPROM.
      if (CONFIGURED_OPERATING_MODE != SELECTED_OPERATING_MODE) {
        // Previously we were a relay interface or just never configured.
        EEPROM.put(EEPROM_CONFIGURED_OPERATING_MODE, SELECTED_OPERATING_MODE);
        EEPROM.write(EEPROM_SOURCE_ADDRESS, DEFAULT_SOURCE_ADDRESS);
      }
      break;
    case RELAY_INTERFACE:
      // We transmit status reports using PGN128006, PGN128007 and
      // PGN128008 and receive commands via N2K's group function
      // mechanism.
      TransmitMessages[0] = 128006UL;
      TransmitMessages[1] = 128007UL;
      TransmitMessages[2] = 128008UL;
      NMEA2000.AddGroupFunctionHandler(new GroupFunctionHandlerForPGN128006(&NMEA2000, &updatePGN128006));
      if (CONFIGURED_OPERATING_MODE != SELECTED_OPERATING_MODE) {
        // Previously we were a switch interface or just never configured.
        EEPROM.put(EEPROM_CONFIGURED_OPERATING_MODE, SELECTED_OPERATING_MODE);
        EEPROM.write(EEPROM_SOURCE_ADDRESS, DEFAULT_SOURCE_ADDRESS);
        PGN128007v.setThrusterMotorType(THRUSTER_MOTOR_TYPE);
        PGN128007v.setMotorPowerRating(THRUSTER_MOTOR_POWER_RATING);
        PGN128007v.setMaximumMotorTemperatureRating(THRUSTER_MAXIMUM_MOTOR_TEMPERATURE_RATING);
        PGN128007v.setMaximumRotationalSpeed(THRUSTER_MAXIMUM_ROTATIONAL_SPEED);
        EEPROM.put(EEPROM_PGN128007, PGN128007v);
        PGN128008v.setTotalMotorOperatingTime(DEFAULT_TOTAL_MOTOR_OPERATING_TIME);
        EEPROM.put(EEPROM_PGN128008, PGN128008v);
      }
      EEPROM.get(EEPROM_PGN128007, PGN128007v);
      break;
    default:
      break;
  }

  // Restore PGN128007 and PGN128008 from EEPROM and assign thruster
  // identifier from DIP switch setting
  EEPROM.get(EEPROM_PGN128007, PGN128007v);
  EEPROM.get(EEPROM_PGN128008, PGN128008v);
  ADDRESS_SWITCH.sample();
  PGN128006v.setThrusterIdentifier(ADDRESS_SWITCH.value());
  PGN128007v.setThrusterIdentifier(ADDRESS_SWITCH.value());
  PGN128008v.setThrusterIdentifier(ADDRESS_SWITCH.value());
  PGN128006c.setThrusterIdentifier(ADDRESS_SWITCH.value());
  #ifdef DEBUG_SERIAL
    PGN128007v.serialDump();
  #endif

  // Initialise all the LEDs //////////////////////////////////////////
  LED_MANAGER.operate(GPIO_BOARD_LED, 0, 3);
  switch (SELECTED_OPERATING_MODE) {
    case SWITCH_INTERFACE: LED_MANAGER.operate(GPIO_POWER_LED, 0, -10000); break;
    case RELAY_INTERFACE: LED_MANAGER.operate(GPIO_POWER_LED, 1); break;
  }

  // Initialise and start N2K services.
  NMEA2000.SetProductInformation(PRODUCT_SERIAL_CODE, PRODUCT_CODE, PRODUCT_TYPE, PRODUCT_FIRMWARE_VERSION, PRODUCT_VERSION);
  NMEA2000.SetDeviceInformation(DEVICE_UNIQUE_NUMBER, DEVICE_FUNCTION, DEVICE_CLASS, DEVICE_MANUFACTURER_CODE);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, EEPROM.read(EEPROM_SOURCE_ADDRESS)); // Configure for sending and receiving.
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.ExtendTransmitMessages(TransmitMessages); // Tell library which PGNs we transmit
  NMEA2000.SetISORqstHandler(&ISORequestHandler);
  NMEA2000.SetMsgHandler(messageHandler);
  NMEA2000.Open();
}

/**********************************************************************
 * MAIN PROGRAM - loop()
 * 
 * With the exception of NMEA2000.parseMessages() all of the functions
 * called from loop() implement interval timers which ensure that they
 * will mostly return immediately, only performing their substantive
 * tasks at intervals defined by program constants.
 * 
 * The global constant JUST_STARTED is used to delay acting on switch
 * inputs until a newly started system has stabilised and the GPIO
 * inputs have been debounced.
 */ 

void loop() {
  static bool JUST_STARTED = true;
  if (JUST_STARTED && (millis() > STARTUP_SETTLE_PERIOD)) {
    #ifdef DEBUG_SERIAL
    Serial.print("Starting (N2K Source address: "); Serial.print(NMEA2000.GetN2kSource()); Serial.println(")");
    Serial.print("Operating mode: "); Serial.println(SELECTED_OPERATING_MODE?"OPERATE":"CONTROL");
    Serial.print("Thruster ID: "); Serial.println(PGN128006v.getThrusterIdentifier());
    #endif
    JUST_STARTED = false;
  }

  switch (SELECTED_OPERATING_MODE) {

    case SWITCH_INTERFACE:
      // Module is a CONTROL device and therefore has switch inputs.
      // which we debounce. If we have just started then we transmit
      // the current (just initialised) status as an initialiser of the
      // remote device, otherwise we process inputs to dynamically
      // update the current status and transmit this to the remote.
      // Finally, we continuously check that the remote thruster is
      // still alive.
      DEBOUNCER.debounce();
      if (JUST_STARTED) {
        transmitThrusterControl();
        // TODO: Ask remote thruster for PGN128007.
      } else {
        processSwitchInputs();
      }
      checkConnection();
      digitalWrite(GPIO_ALARM_INDICATOR_RELAY, checkEvents());
      break;
    case RELAY_INTERFACE: // Module is an OPERATING device
      if (PGN128006v.getPowerEnable() == N2kDD002_On) {
        switch (PGN128006v.getThrusterDirectionControl()) {
          case N2kDD473_ThrusterToPORT:
            digitalWrite(GPIO_PORT_RELAY, 1);
            digitalWrite(GPIO_COMMON_RELAY, 1);
            digitalWrite(GPIO_STARBOARD_RELAY, 0);
            checkTimeout((unsigned long) PGN128006v.getCommandTimeout() * 1000);
            if (THRUSTER_START_TIME == 0UL) THRUSTER_START_TIME = millis();
            break;
          case N2kDD473_ThrusterToSTARBOARD:
            digitalWrite(GPIO_STARBOARD_RELAY, 1);
            digitalWrite(GPIO_COMMON_RELAY, 1);
            digitalWrite(GPIO_PORT_RELAY, 0);
            checkTimeout((unsigned long) PGN128006v.getCommandTimeout() * 1000);
            if (THRUSTER_START_TIME == 0UL) THRUSTER_START_TIME = millis();
            break;
          default:
            digitalWrite(GPIO_PORT_RELAY, 0);
            digitalWrite(GPIO_STARBOARD_RELAY, 0);
            digitalWrite(GPIO_COMMON_RELAY, 0);
            break;
        }
      } else {
        digitalWrite(GPIO_PORT_RELAY, 0);
        digitalWrite(GPIO_STARBOARD_RELAY, 0);
        digitalWrite(GPIO_COMMON_RELAY, 0);
      }
      // If we have stopped receiving operating commands, then stop
      // thrusters and update and save total run time.
      if (checkTimeout(0UL)) {
        PGN128006v.setThrusterDirectionControl(N2kDD473_OFF);
        PGN128008v.bumpTotalMotorOperatingTime((unsigned long) (millis() - THRUSTER_START_TIME) / 1000);
        EEPROM.put(EEPROM_PGN128008, PGN128008v);
      }
      transmitStatus();
      break;
    default:
      break;
  }
  
  // Process any received messages.
  NMEA2000.ParseMessages();
  // The above call may have resulted in acquisition of a new source
  // address, so we check if there has been a change and if so save the
  // new address to EEPROM for future re-use.
  if (NMEA2000.ReadResetAddressChanged()) EEPROM.update(EEPROM_SOURCE_ADDRESS, NMEA2000.GetN2kSource());

  
  // Update the states of connected LEDs
  LED_MANAGER.loop();
}

///////////////////////////////////////////////////////////////////////
// START OF CONTROL MODE FUNCTIONS
///////////////////////////////////////////////////////////////////////

/**********************************************************************
 * processSwitchInputs() should be called directly from loop() to
 * update the properties of PGN128006c from switch and sensor inputs. If a
 * switch input is active (i.e. a joystick or whatever is being
 * operated) the function uses a simple elapse timer to repeatedly
 * transmit direction controls to the configured remote thruster at the
 * frequency determined by the control update frequency.
 */
void processSwitchInputs() {
  static unsigned long deadline = 0UL;
  unsigned long now = millis();
  bool transmit = false;

  if ((PGN128006_CommandTransmitInterval > 0) && (now > deadline)) {

    if (!DEBOUNCER.channelState(GPIO_PORT_SWITCH) && DEBOUNCER.channelState(GPIO_STARBOARD_SWITCH)) {
      PGN128006c.setThrusterDirectionControl(N2kDD473_ThrusterToPORT);
      PGN128006c.setPowerEnable(N2kDD002_On);
      processAnalogControlInputs();
      transmit = true;
    }

    if (!DEBOUNCER.channelState(GPIO_STARBOARD_SWITCH) && DEBOUNCER.channelState(GPIO_PORT_SWITCH)) {
      PGN128006c.setThrusterDirectionControl(N2kDD473_ThrusterToSTARBOARD);
      PGN128006c.setPowerEnable(N2kDD002_On);
      processAnalogControlInputs();
      transmit = true;;
    }
    
    tN2kDD474 thrusterRetractControl = (!DEBOUNCER.channelState(GPIO_RETRACT_SWITCH))?N2kDD474_Extend:N2kDD474_Retract;
    if (PGN128006c.getThrusterRetractControl() != thrusterRetractControl) {
      PGN128006c.setThrusterRetractControl(thrusterRetractControl);
      transmit = true;
    }

    if (transmit) transmitThrusterControl();

    deadline = (now + PGN128006_CommandTransmitInterval);
  }
}

/**********************************************************************
 * Read analog inputs for speed and azimuth and assign to the
 * appropriate PGN128006 properties.
 */
void processAnalogControlInputs() {
  int value = adc->analogRead(GPIO_SPEED_ANALOG_IN);
  if (value != ADC_ERROR_VALUE) {
    PGN128006c.setSpeedControl((uint8_t) ((value / adc->adc0->getMaxValue()) * 100));
  }
  value = adc->analogRead(GPIO_AZIMUTH_ANALOG_IN);
  if (value != ADC_ERROR_VALUE) {
    PGN128006c.setAzimuthControl((value / adc->adc0->getMaxValue()) * 100);
  }
}

/**********************************************************************
 * transmitThrusterControl() transmits a PGN 126208 Command Group
 * Function to the thruster identified by THRUSTER_SOURCE_ADDRESS using
 * the PGN128006c variable as the command property source.
 * 
 * The N2K specification requires that ThrusterIdentifier,
 * ThrusterDirectionControl, PowerEnable and SpeedControl fields are
 * always transmitted, Other fields will only be transmitted if their
 * value has changed since the last transmit.
 */
void transmitThrusterControl() {
  if (ADDRESS_SWITCH.value() != BROADCAST_SOURCE_ADDRESS) {
    tN2kMsg N2kMsg;
    N2kMsg.SetPGN(126208UL);                // Command Group Function 
    N2kMsg.Priority = 2;                    // High priority
    N2kMsg.Destination = THRUSTER_SOURCE_ADDRESS;   // Send direct to the configured thruster module
    N2kMsg.AddByte(0x01);                   // This is a command message
    N2kMsg.Add3ByteInt(128006UL);           // Thruster Control Status PGN
    N2kMsg.AddByte(0xF8);                   // Retain existing priority

    PGN128006c.setDirty(2);
    PGN128006c.setDirty(3);
    PGN128006c.setDirty(4);
    PGN128006c.setDirty(6);

    int dirtyCount = 0;
    for (int i = 2; i <= PGN128006_FieldCount; i++) if (PGN128006c.isDirty(i)) dirtyCount++;

    if (dirtyCount > 0) {
      N2kMsg.AddByte(dirtyCount);                   // Number of parameter pairs
      for (int i = 2; i<= PGN128006_FieldCount; i++) {
        switch (i) {
          case 2:
            if (PGN128006c.isDirty(i)) {
              N2kMsg.AddByte(i);                   // Parameter 1 - Field 2 (Thruster Identifier)
              N2kMsg.AddByte(PGN128006c.getThrusterIdentifier());
              PGN128006c.setClean(i);
            }
            break;
          case 3:
            if (PGN128006c.isDirty()) {
              N2kMsg.AddByte(i);                   // Parameter 2 - Field 3 (Thruster Direction Control)
              N2kMsg.AddByte(PGN128006c.getThrusterDirectionControl());
              PGN128006c.setClean(i);
            break;
          case 4:
            if (PGN128006c.isDirty(i)) {
              N2kMsg.AddByte(i);                   // Parameter 3 - Field 4 Power Enable
              N2kMsg.AddByte(PGN128006c.getPowerEnable());
              PGN128006c.setClean(i);
            }
            break;
          case 5:
            if (PGN128006c.isDirty(i)) {
              N2kMsg.AddByte(0x05);                   // Parameter 4 - Field 5 (Thruster Retract Control)
              N2kMsg.AddByte(PGN128006c.getThrusterRetractControl());
              PGN128006c.setClean(i);
            }
            break;
          case 6:
            if (PGN128006c.isDirty(i)) {
              N2kMsg.AddByte(0x06);                   // Parameter 5 - Field 6 (Speed Control)
              N2kMsg.AddByte(PGN128006c.getSpeedControl());
              PGN128006c.setClean(i);
            }
            break;
          case 7:
            if (PGN128006c.isDirty(i)) {
              N2kMsg.AddByte(0x07);                   // Parameter 6 - Field 7 (Thruster Control Events)
              N2kMsg.AddByte(PGN128006c.getThrusterControlEvents().Events);
              PGN128006c.setClean(i);
            }
            break;
          case 8:
            if (PGN128006c.isDirty(i)) {
              N2kMsg.AddByte(0x08);                   // Parameter 7 - Field 8 (Command Timeout Interval)
              N2kMsg.Add1ByteDouble(PGN128006c.getCommandTimeout(), 0.005);
              PGN128006c.setClean(i);
            }
            break;
          case 9:
            if (PGN128006c.isDirty(i)) {
              N2kMsg.AddByte(0x09);                   // Parameter 8 - Field 9 (Azimuth Control)
              N2kMsg.Add2ByteDouble(PGN128006c.getAzimuthControl(), 0.0001);
              PGN128006c.setClean(i);
            }
            break;
          default:
            break;
        }
      }
      NMEA2000.SendMsg(N2kMsg);
      LED_MANAGER.operate(GPIO_POWER_LED, 1, 1);
    }
    }
  }
}

/**********************************************************************
 * PGN128006Handler() processes PGN 128006 (Thruster Control Status)
 * messages looking for a message from the thruster configured by the
 * hardware DIP switch.  When such a message is found, the power LED is
 * set to steady to indicate that the configured thruster has been
 * identified on the network and the N2K source address of the message
 * sender is copied into THRUSTER_SOURCE_ADDRESS thus signalling that
 * thruster control messages can be sent. 
 */
void PGN128006Handler(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char ThrusterIdentifier;
  tN2kThrusterDirectionControl ThrusterDirectionControl;
  tN2kGenericStatusPair PowerEnable;
  tN2kThrusterRetraction ThrusterRetractControl;
  unsigned char SpeedControl;
  tN2kThrusterControlEvents ThrusterControlEvents;
  double CommandTimeout;
  double AzimuthControl;

  if (ParseN2kPGN128006(N2kMsg, SID, ThrusterIdentifier, ThrusterDirectionControl, PowerEnable, ThrusterRetractControl, SpeedControl, ThrusterControlEvents, CommandTimeout, AzimuthControl)) {
    if (ThrusterIdentifier == ADDRESS_SWITCH.value()) {
      THRUSTER_SOURCE_ADDRESS = (unsigned char) N2kMsg.Source;
      THRUSTER_SOURCE_ADDRESS_UPDATE_TIMESTAMP = millis();
      PGN128007v.setSID(SID);
      PGN128007v.setThrusterIdentifier(ThrusterIdentifier);
      PGN128006v.setThrusterDirectionControl(ThrusterDirectioControl);
      PGN128006v.setPowerEnable(PowerEnable);
      PGN128006v.setThrusterRetractControl(ThrusterRetractControl);
      PGN128006v.setSpeedControl(SpeedControl);
      PGN128006v.setThrusterControlEvents(ThrusterControlEvents);
      PGN128006v.setCommandTimeout(CommandTimeout);
      PGN128006.AzimuthControl(AzimuthControl);
      LED_MANAGER.operate(GPIO_POWER_LED, 1, 0);
    }
    // TODO: Validate these against PGN128007c.
  }
}

/**********************************************************************
 * PGN128007Handler() accepts PGN 128007 (Thruster Information)
 * messages, accepting only those that originate from a thruster with
 * an identifier that matches the hardware DIP switch address and
 * unpacking received properties into PGN128007v.
 */ 
PGN128007Handler(const tN2kMsg &N2kMsg) {
  uint8_t ThrusterIdentifier;
  DD487 MotorPowerType;
  int MotorPowerRating,
  double MaximumMotorTemperatureRating;
  double MaximumRotationalSpeed;

  if (ParseN2kPGN128007(N2kMsg, ThrusterIdentifier, MotorPowerType, MotorPowerRating, MaximumMotorTemperatureRating, MaximumRotationalSpeed)) {
    if (ThrusterIdentifier == ADDRESS_SWITCH.value()) {
      PGN128007v.setThrusterIdentifier(ThrusterIdentifier);
      PGN128007v.setMotorPowerType(MotorPowerType);
      PGN128007v.setMotorPowerRating(MotorPowerRating);
      PGN128007v.setMaximumMotorTemperatureRating = MaximumMotorTemperatureRating;
      PGN128007v.setMaximumRotationalSpeed = MaximumRotationalSpeed();
    } 
  }
}

/**********************************************************************
 * PGN128008Handler() accepts PGN 128008 (Thruster Motor Status)
 * messages, accepting only those that originate from a thruster with
 * an identifier that matches the hardware DIP switch address. Field
 * values are unpacked into PGN128008v.
 */
void PGN128008Handler(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char ThrusterIdentifier;
  tN2kDD471 ThrusterMotorEvents;
  unsigned char MotorCurrent;
  double MotorTemperature;
  unsigned int TotalMotorOperatingTime;
  bool alarm = false;

  if (ParseN2kPGN128008(N2kMsg, SID, ThrusterIdentifier, ThrusterMotorEvents, MotorCurrent, MotorTemperature, TotalMotorOperatingTime)) {
    if (ThrusterIdentifier == ADDRESS_SWITCH.value()) {
      PGN128008v.setSID(SID);
      PGN128008v.setThrusterMotorEvents(ThrusterMotorEvents);
      PGN128008v.setMotorCurrent(MotorCurrent);
      PGN128008v.setMotorTemperature(MotorTemperature);
      PGN128008v.setTotalMotorOperatingTime(TotalMotorOperatingTime);
    }
  }
}

/**********************************************************************
 * checkConnection() should be called directly from loop(). It tries to
 * make sure that a remote thruster connection is still viable by
 * checking that PGN128006Handler() has received status updates from
 * the remote within the period defined by THRUSTER_SOURCE_ADDRESS_TIMEOUT.
 * If the status updates have dried up then THRUSTER_SOURCE_ADDRESS is
 * reset to the broadcast address and the power LED is returned to
 * continuous flashing to indicate no connection.
 */
void checkConnection() {
  if (millis() > (THRUSTER_SOURCE_ADDRESS_UPDATE_TIMESTAMP + THRUSTER_SOURCE_ADDRESS_TIMEOUT)) {
    if (THRUSTER_SOURCE_ADDRESS != BROADCAST_SOURCE_ADDRESS) {
      THRUSTER_SOURCE_ADDRESS = BROADCAST_SOURCE_ADDRESS;
      LED_MANAGER.operate(GPIO_POWER_LED, 0, 10000);
    }
  }
}

/**********************************************************************
 * checkEvents() should be called directly from loop. It examines PGNs
 * received from the controlled thruster, looking for event field
 * problems, returning true if problems are found otherwise false.
 */
void checkEvents() {
  bool retval = false;
  if ((PGN128006v.getThrusterIdentifier() != INSTANCE_UNDEFINED) && (PGN128008v.getThrusterIdentifier() != INSTANCE_UNDEFINED)) {
    // We have received all status PGNs from the remote thruster,d so we
    // can check 128006 and 128008 against 128007.
    if (PGN128006v.getThrusterControlEvents() || PGN128008v.getThrusterMotorEvents().Events()) retval = true;
    return(retval);
  }
} 

///////////////////////////////////////////////////////////////////////
// END OF SWITCH MODE FUNCTIONS
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
// START OF RELAY MODE FUNCTIONS
///////////////////////////////////////////////////////////////////////

/**********************************************************************
 * transmitStatus() should be called directly from loop(). It arranges
 * to transmit PGNs 128007, 128007 and 128008 at the frequencies set by
 * their current, respective, operating intervals and makes sure that
 * each operating interval is adjusted to reflect the current thruster
 * operating state (PGNs are issued more frequently when thrusters are
 * operating). PGNs transmitted at the same time are all assigned the
 * same SID. 
 */
void transmitStatus() {
  unsigned char SID = 0;

  static unsigned long PGN128006Deadline = 0UL;
  static unsigned long PGN128007Deadline = 0UL;
  static unsigned long PGN128008Deadline = 0UL;
  unsigned long now = millis();

  if ((PGN128006_UPDATE_INTERVAL != 0UL) && (now > PGN128006Deadline)) {
    transmitPGN128006(SID);
    PGN128006_UPDATE_INTERVAL = isOperating()?PGN128006_DynamicUpdateInterval:PGN128006_StaticUpdateInterval;
    PGN128006Deadline = (now + PGN128006_UPDATE_INTERVAL);
  }

  if ((PGN128007_UPDATE_INTERVAL != 0UL) && (now > PGN128007Deadline)) {
    transmitPGN128007(SID);
    PGN128007_UPDATE_INTERVAL = isOperating()?PGN128007_DynamicUpdateInterval:PGN128007_StaticUpdateInterval;
    PGN128007Deadline = (now + PGN128007_UPDATE_INTERVAL);
  }

  if ((PGN128008_UPDATE_INTERVAL != 0UL) && (now > PGN128008Deadline)) {
    transmitPGN128008(SID);
    PGN128008_UPDATE_INTERVAL = isOperating()?PGN128008_DynamicUpdateInterval:PGN128008_StaticUpdateInterval;
    PGN128008Deadline = (now + PGN128008_UPDATE_INTERVAL);
  }

  SID++;
}

/**********************************************************************
 * transmitPGN128006() provisions a Thruster Control Status message
 * with values drawn from the current state and transmits it using the
 * supplied SID (sequence id).
 */
void transmitPGN128006(unsigned char SID) {
  #ifdef DEBUG_SERIAL
  Serial.println("TRANSMITTING PGN128006");
  #endif
  tN2kMsg N2kMsg;

  SetN2kPGN128006(N2kMsg, SID, PGN128006v.getThrusterIdentifier(), PGN128006v.getThrusterDirectionControl(), PGN128006v.getPowerEnable(), PGN128006v.getThrusterRetractControl(), PGN128006v.getSpeedControl(), PGN128006v.getThrusterControlEvents(), PGN128006v.getCommandTimeout(), PGN128006v.getAzimuthControl());
  NMEA2000.SendMsg(N2kMsg);
}

/**********************************************************************
 * transmitPGN128007() provisions a Thruster Information message with
 * values drawn from the current state and transmits it (the supplied
 * SID (sequence id) is not specified for this PGN).
 */
void transmitPGN128007(unsigned char SID) {
  #ifdef DEBUG_SERIAL
  Serial.println("TRANSMITTING PGN128007");
  #endif
  tN2kMsg N2kMsg;

  SetN2kPGN128007(N2kMsg, PGN128007v.getThrusterIdentifier(), PGN128007v.getThrusterMotorType(), PGN128007v.getMotorPowerRating(), PGN128007v.getMaximumMotorTemperatureRating(), PGN128007v.getMaximumRotationalSpeed());
  NMEA2000.SendMsg(N2kMsg);
}

/**********************************************************************
 * transmitPGN128008() provisions a Thruster Motor Status message with
 * values drawn from the current state and transmits it using the
 * supplied SID (sequence id).
 */
void transmitPGN128008(unsigned char SID) {
  #ifdef DEBUG_SERIAL
  Serial.println("TRANSMITTING PGN128008");
  #endif
  tN2kMsg N2kMsg;

  SetN2kPGN128008(N2kMsg, SID, PGN128008v.getThrusterIdentifier(), PGN128008v.getThrusterMotorEvents(), PGN128008v.getMotorCurrent(), PGN128008v.getMotorTemperature(), PGN128008v.getTotalMotorOperatingTime());
  NMEA2000.SendMsg(N2kMsg);
}

/**********************************************************************
 * isOperating() is a convenience function which returns true if the
 * thruster is being commanded to operate and otherwise false.
 */
bool isOperating() {
  return(PGN128006v.getPowerEnable() == N2kDD002_On) && ((PGN128006v.getThrusterDirectionControl() == N2kDD473_ThrusterToPORT) || (PGN128006v.getThrusterDirectionControl() == N2kDD473_ThrusterToSTARBOARD)));
}

/**********************************************************************
 * checkTimeout() manages the automatic stopping of a thruster when
 * operating control commands dry up. It has two call conventions.
 * 
 * checkTimeout(0UL) should be called directly from loop(). It will
 * return true if a previously set timeout period has expired.
 * 
 * checkTimeout(t) can be called to establish a timeout of <t> milli-
 * seconds. The function returns false.
 */
bool checkTimeout(unsigned long timeout) {
  static unsigned long deadline = 0UL;
  unsigned long now = millis();
  bool retval = false;

  if (timeout == 0UL) {
    if ((deadline != 0UL) && (now > deadline)) {
      deadline = 0UL;
      retval = true;
    }
  } else {
    deadline = (now + timeout);
  }
  return(retval);
}

/**********************************************************************
 * updatePGN128006() is used as a callback by handlers in the
 * GroupFunctionHandlerForPGN128006 class. It accepts a sparse array
 * of values for properties in the eponymous PGN and, if a value is
 * marked as modified, it updates the local property with the supplied
 * value iff the ThrusterIdentifier in the update array matches the
 * identifier defined for this module.
 */
void updatePGN128006(PGN128006_Field fields[]) {
  bool canUpdate = false;
  
  for (int i = 0; i < (PGN128006_FieldCount + 1); i++) {
    switch (i) {
      case 2:
        if (fields[i].dirty && (PGN128006v.getThrusterIdentifier() == fields[i].value.F02)) canUpdate = true;
        break;
      case 3:
        if (canUpdate && fields[i].dirty) PGN128006v.setThrusterDirectionControl(fields[i].value.F03);
        break;
      case 4:
        if (canUpdate && fields[i].dirty) PGN128006v.setPowerEnable(fields[i].value.F04);
        break;
      case 5:
        if (canUpdate && fields[i].dirty) PGN128006v.setThrusterRetractControl(fields[i].value.F05);
        break;
      case 6:
        if (canUpdate && fields[i].dirty) PGN128006v.setSpeedControl(fields[i].value.F06);
        break;
      case 7:
        if (canUpdate && fields[i].dirty) PGN128006v.setThrusterControlEvents(fields[i].value.F07);
        break;
      case 8:
        if (canUpdate && fields[i].dirty) PGN128006v.setCommandTimeout(fields[i].value.F08);
        break;
      case 9:
        if (canUpdate && fields[i].dirty) PGN128006v.setAzimuthControl(fields[i].value.F09);
        break;
      default:
        break;
    }
  }
  // From the N2k specification: "In addition to the required
  // Acknowledge Group Function, this PGN shall be sent as response
  // to any Command Group Function."
  transmitPGN128006();
}

/**********************************************************************
 * Some MFDs issue PGN 059904 ISO Request PGNs in order to get data
 * from a device, so we might as well pander to them.
 */
bool ISORequestHandler(unsigned long RequestedPGN, unsigned char Requester, int DeviceIndex) {
  if (SELECTED_OPERATING_MODE == RELAY_INTERFACE) {
    switch (RequestedPGN) {
      case 128006UL: transmitPGN128006(0); return(true); break;
      case 128007UL: transmitPGN128007(0); return(true); break;
      case 128008UL: transmitPGN128008(0); return(true); break;
      default: break;
    }
  }
  return(false);
}

///////////////////////////////////////////////////////////////////////
// END OF OPERATOR MODE FUNCTIONS
///////////////////////////////////////////////////////////////////////

/**********************************************************************
 * Processes incoming messages using the NMEA2000Handlers jump vector
 * to invoke handlers registered for specific PGNs.
 */
void messageHandler(const tN2kMsg &N2kMsg) {
  int iHandler;
  for (iHandler = 0; (NMEA2000Handlers[iHandler].PGN != 0) && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}

#ifdef DEBUG_SERIAL

#endif
