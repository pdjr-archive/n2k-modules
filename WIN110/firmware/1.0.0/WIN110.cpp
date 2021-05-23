/**********************************************************************
 * winctl-1.0.0.cpp - WINCTL firmware version 1.0.0.
 * Copyright (c) 2020 Paul Reeve, <preeve@pdjr.eu>
 *
 * This firmware allows control of one or two remote windlasses using
 * the NMEA 2000 Windlass Network Messages protocol based around PGNs
 * 128776, 128777 and 128778.
 * 
 * The firmware supports two control channels W0 and W1. Each control
 * channel is configured by a single unsigned byte which supplies the
 * instance number of the remote windlass that should be associated
 * with the channel. These instance numbers are recovered from EEPROM
 * when the firmware boots. A set of seven GPIO inputs allow the
 * connection of a seven bit DIP switch that can be used to specify
 * an instance number. A further two GPIO inputs save the configured
 * instance number to EEPROM when set high.
 * 
 * The firmware continuously monitors the NMEA bus for PGN 128777
 * Windlass Operating Status messages decorated with an instance number
 * that corresponds to one or other of the configured values. These
 * messages are used to (i) update the firmware's notion of the NMEA
 * network address of each configured remote windlass (allowing control
 * messages to be directed to the remote device) and (ii) condition
 * the status output channels associated with a particular channel.
 * Continuous update of remote windlass network addresses is required
 * because the NMEA bus may require nodes to dynamically change their
 * network address.
 *  
 * UP and DOWN GPIO inputs are provided for each control channel: if a
 * control channel becomes active then the firmware will output a
 * PGN 126208 Group Function Control message commanding the associated
 * windlass to operate in the same sense as the control. PGN 126208
 * messages will continue to be output every 250ms until the control
 * channel becomes inactive.
 * 
 * Five GPIO status outputs are provided (conditioned by PGN 128777
 * Windlass Operating Status messages).
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <NMEA2000_CAN.h>
#include <N2kTypes.h>
#include <N2kMessages.h>
#include <Debouncer.h>
#include <LedManager.h>
#include <DilSwitch.h>
#include <WindlassState.h>
#include <Windlass.h>
#include <arraymacros.h>
#include <PGN128776.h>
#include <PGN128777.h>
#include <PGN128778.h>
#include <SW6RL4.h>
#include "GroupFunctionHandlers.h"

/**********************************************************************
 * DEBUG AND TESTING
 * 
 * Defining DEBUG_SERIAL includes the function debugDump() and arranges
 * for it to be called from loop() every DEBUG_SERIAL_INTERVAL ms.
 * 
 * Defining DEBUG_USE_DEBUG_ADDRESSES disables normal instance number
 * recovery from EEPROM and remote windlass address recovery from the
 * network and instead forces the use of the values defined here. 
 */

#define DEBUG_SERIAL
#define DEBUG_SERIAL_START_DELAY 4000
#define DEBUG_SERIAL_INTERVAL 1000UL

#define DEBUG_USE_DEBUG_ADDRESSES
#define DEBUG_W0_INSTANCE_VALUE 0x22
#define DEBUG_W0_ADDRESS_VALUE 0x22
#define DEBUG_W1_INSTANCE_VALUE 127
#define DEBUG_W1_ADDRESS_VALUE 255

/**********************************************************************
 * MCU EEPROM STORAGE DEFINITIONS
 * 
 * These two addresses specify the persistent storage address that
 * should be used to store the 1-byte remote windlass instance numbers. 
 */
#define EEPROM_CONFIGURED_OPERATING_MODE 0 // 1 byte
#define EEPROM_WINDLASS0 100
#define EEPROM_WINDLASS1 300


/**********************************************************************
 * GPIOs USED IN SWITCH AND RELAY MODES
 */
#define GPIO_POWER_LED GPIO_LED_2

/**********************************************************************
 * GPIOs USED IN SWITCH MODE
 */
#define GPIO_PROGRAM_SWITCH PUSH_1
#define GPIO_W0_UP_SWITCH GPIO_SWITCH_1
#define GPIO_W0_DN_SWITCH GPIO_SWITCH_2
#define GPIO_W1_UP_SWITCH GPIO_SWITCH_3
#define GPIO_W1_DN_SWITCH GPIO_SWITCH_4
#define GPIO_W0_UP_INDICATOR GPIO_RELAY_1
#define GPIO_W0_DN_INDICATOR GPIO_RELAY_2
#define GPIO_W1_UP_INDICATOR GPIO_RELAY_3
#define GPIO_W1_DN_INDICATOR GPIO_RELAY_4

/**********************************************************************
 * GPIOs USED IN RELAY MODE
 */
#define GPIO_W0_DOCKED_SENSOR GPIO_SWITCH_1
#define GPIO_W0_DEPLOYED_SENSOR GPIO_SWITCH_2
#define GPIO_W0_RETRIEVING_SENSOR GPIO_SWITCH_3
#define GPIO_W0_DEPLOYING_SENSOR GPIO_SWITCH_4
#define GPIO_W0_ROTATION_SENSOR GPIO_SWITCH_5
#define GPIO_W0_STRESS_SENSOR GPIO_SWITCH_6
#define GPIO_W0_UP_RELAY GPIO_RELAY_1
#define GPIO_W0_DN_RELAY GPIO_RELAY_2

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

#define DEVICE_CLASS 30
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
#define PRODUCT_FIRMWARE_VERSION "1.0.0 (Sep 2020)"
#define PRODUCT_LEN 1
#define PRODUCT_N2K_VERSION 2101
#define PRODUCT_SERIAL_CODE "002-849" // PRODUCT_CODE + DEVICE_UNIQUE_NUMBER
#define PRODUCT_TYPE "WINCTL"
#define PRODUCT_VERSION "1.0 (Sep 2020)"

/**********************************************************************
 * Include the build.h header file which can be used to override any or
 * all of the above  constant definitions.
 */

#include "build.h"

#define DEFAULT_SOURCE_ADDRESS 22         // Seed value for address claim
#define INSTANCE_UNDEFINED 255            // Flag value
#define INSTANCE_DISABLED 127             // Flag value
#define STARTUP_SETTLE_PERIOD 5000        // Wait this many ms before processing switch inputs
#define OPERATE_SWITCH_PROCESS_INTERVAL 250       // Process UP and DN switch inputs every 250 ms
#define PROGRAM_SWITCH_PROCESS_INTERVAL 330       // Process the PRG switch every 330 ms
#define SENSOR_PROCESS_INTERVAL         100
#define REMOTE_WINDLASS_CHECK_INTERVAL   10000     // Check 
#define RELAY_UPDATE_INTERVAL 330         // Update outputs every n ms
#define LED_MANAGER_HEARTBEAT 100         // Settings for LEDs on module case
#define LED_MANAGER_INTERVAL 10           //
#define PULSE_DISTANCE 0.1                // In metres

/**********************************************************************
 * Declarations of local functions.
 */

#ifdef DEBUG_SERIAL
void debugDump();
#endif
unsigned char getPoleInstance();
void messageHandler(const tN2kMsg&);
void PGN128777(const tN2kMsg&);
void processSwitches(WindlassState **windlasses);
void transmitWindlassControl(WindlassState *windlass);
void operateOutputs(WindlassState *windlass);

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
 * Create a switch debouncer DEBOUNCER and associate with it the GPIO
 * pins that are connected to switches.
 */

int SWITCHES[DEBOUNCER_SIZE] = { GPIO_W0_PRG_SWITCH, GPIO_W0_UP_SWITCH, GPIO_W0_DN_SWITCH, GPIO_W1_PRG_SWITCH, GPIO_W1_UP_SWITCH, GPIO_W1_DN_SWITCH, -1, -1 };
Debouncer DEBOUNCER (SWITCHES);

/**********************************************************************
 * Create an LED manager with operating characteristics that suit the
 * status LED mounted on the module case.
 */
LedManager LED_MANAGER (LED_MANAGER_HEARTBEAT, LED_MANAGER_INTERVAL);

enum OPERATING_MODE_TYPE { SWITCH_INTERFACE, RELAY_INTERFACE, UNKNOWN } SELECTED_OPERATING_MODE = UNKNOWN;

unsigned long PGN128776_UPDATE_INTERVAL = PGN128776_StaticUpdateInterval;
unsigned long PGN128777_UPDATE_INTERVAL = PGN128777_StaticUpdateInterval;
unsigned long PGN128778_UPDATE_INTERVAL = PGN128778_StaticUpdateInterval;

Windlass WINDLASS0 = Windlass();
Windlass WINDLASS1 = Windlass();

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
  for (unsigned int i = 0 ; i < ELEMENTCOUNT(ipins); i++) { pinMode(ipins[i], INPUT_PULLUP); }
  for (unsigned int i = 0 ; i < ELEMENTCOUNT(opins); i++) { pinMode(opins[i], OUTPUT); }

  // Get the user-selected operating mode and the mode that was last configured.
  SELECTED_OPERATING_MODE = (!digitalRead(GPIO_MODE))?SWITCH_INTERFACE:RELAY_INTERFACE;
  OPERATING_MODE_TYPE CONFIGURED_OPERATING_MODE = EEPROM.read(EEPROM_CONFIGURED_OPERATING_MODE);
 
  // Configure the module to suit the selected operating mode. If the
  // operating mode has been changed, then this may require a bit of
  // effort...
  switch (SELECTED_OPERATING_MODE) {
    case SWITCH_INTERFACE:
      // Set the PGNs of messages we transmit and receive.
      TransmitMessages[0] = 126208UL;
      NMEA2000Handlers[0] = { 128776UL, &PGN128776Handler };
      // If the configured operating mode is not the selected operating
      // mode then we need to initialise the two windlass definitions
      // and save them to EEPROM.
      if (CONFIGURED_OPERATING_MODE != SELECTED_OPERATING_MODE) {
        EEPROM.put(EEPROM_CONFIGURED_OPERATING_MODE, SELECTED_OPERATING_MODE);
        WINDLASS0.setWindlassIdentifier(ADDRESS_SWITCH.sample().value());
        WINDLASS0.saveToEEPROM(EEPROM_WINDLASS0);
        WINDLASS1.setWindlassIdentifier(BROADCAST_ADDRESS);
        WINDLASS1.saveToEEPROM(EEPROM_WINDLASS1);
      }
      WINDLASS0.loadFromEEPROM(EEPROM_WINDLASS0);
      WINDLASS1.loadFromEEPROM(EEPROM_WINDLASS1);
      if (WINDLASS1.getWindlassIdentifier() == WINDLASS0.getWindlassIdentifier()) {
        WINDLASS1.setWindlassIdentifier(255);
      }
      break;
    case RELAY_INTERFACE:
      TransmitMessages[0] = 128776UL;
      TransmitMessages[1] = 128777UL;
      TransmitMessages[2] = 128778UL;
      if (CONFIGURED_OPERATING_MODE != SELECTED_OPERATING_MODE) {
        // Previously we were a switch interface or just never configured.
        EEPROM.put(EEPROM_CONFIGURED_OPERATING_MODE, SELECTED_OPERATING_MODE);
        WINDLASS0.setWindlassIdentifier(ADDRESS_SWITCH.sample().value());
        
        PGN128007v.setThrusterMotorType(THRUSTER_MOTOR_TYPE);
        PGN128007v.setMotorPowerRating(THRUSTER_MOTOR_POWER_RATING);
        PGN128007v.setMaximumMotorTemperatureRating(THRUSTER_MAXIMUM_MOTOR_TEMPERATURE_RATING);
        PGN128007v.setMaximumRotationalSpeed(THRUSTER_MAXIMUM_ROTATIONAL_SPEED);
        EEPROM.put(EEPROM_PGN128007, PGN128007v);
        PGN128008v.setTotalMotorOperatingTime(DEFAULT_TOTAL_MOTOR_OPERATING_TIME);
        EEPROM.put(EEPROM_PGN128008, PGN128008v);
      }
      WINDLASS0.loadFromEEPROM(EEPROM_WINDLASS0);
      attachInterrupt(GPIO_W0_ROTATION_SENSOR, onPulse, FALLING);
      break;
    default:
      break;
  }
  
  #ifdef DEBUG_SERIAL
    WINDLASS0.serialDump();
    WINDLASS1.serialDump();
  #endif
  
  STATUS_LED_MANAGER.operate(GPIO_BOARD_LED, 0, 3);

  NMEA2000.SetProductInformation(PRODUCT_SERIAL_CODE, PRODUCT_CODE, PRODUCT_TYPE, PRODUCT_FIRMWARE_VERSION, PRODUCT_VERSION);
  NMEA2000.SetDeviceInformation(DEVICE_UNIQUE_NUMBER, DEVICE_FUNCTION, DEVICE_CLASS, DEVICE_MANUFACTURER_CODE);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, EEPROM.read(EEPROM_SOURCE_ADDRESS)); // Configure for sending and receiving.
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.ExtendTransmitMessages(TransmitMessages); // Tell library which PGNs we transmit
  NMEA2000.SetISORqstHandler(&ISORequestHandler);
  NMEA2000.AddGroupFunctionHandler(new GroupFunctionHandlerForPGN128006(&NMEA2000, &updatePGN128006));
  NMEA2000.SetMsgHandler(messageHandler);
  NMEA2000.Open();
}

/**********************************************************************
 * MAIN PROGRAM - loop()
 * 
 * With the exception of NMEA2000.parseMessages() all of the functions
 * called from loop() implement interval timers which ensure that they
 * will mostly return immediately, only performing their substantive
 * tasks at the defined intervals.
 * 
 * The global constant JUST_STARTED is used to delay acting on switch
 * inputs until a newly started system has stabilised and the GPIO
 * inputs have been debounced.
 */ 

void loop() {
  static bool JUST_STARTED = true;
  if (JUST_STARTED && (millis() > STARTUP_SETTLE_PERIOD)) {
    JUST_STARTED = false;
    #ifdef DEBUG_SERIAL
    //Serial.print("Starting (N2K Source address: "); Serial.print(NMEA2000.GetN2kSource()); Serial.println(")");
    //Serial.print("Operating mode: "); Serial.println(SELECTED_OPERATING_MODE?"OPERATE":"CONTROL");
    //Serial.print("Thruster ID: "); Serial.println(PGN128006v.getThrusterIdentifier());
    #endif
  }

  switch (SELECTED_OPERATING_MODE) {
    case SWITCH_INTERFACE:
      DEBOUNCER.debounce();
      if (JUST_STARTED) { 
        if (WINDLASS0.getWindlassIdentifier() != INSTANCE_UNDEFINED) transmitWindlassControl(WINDLASS0);
        if (WINDLASS1.getWindlassIdentifier() != INSTANCE_UNDEFINED) transmitWindlassControl(WINDLASS1);
      } else {
        processSwitches();
        operateIndicators(WINDLASS0, GPIO_W0_UP_INDICATOR, GPIO_W0_DN_INDICATOR);
        operateIndicators(WINDLASS1, GPIO_W1_UP_INDICATOR, GPIO_W1_DN_INDICATOR);
      }
      checkConnection();
      break;
    case RELAY_INTERFACE:
      processSensorInputs();
      setRelays(WINDLASS0, GPIO_W0_UP_RELAY, GPIO_W0_DN_RELAY);
      checkTimeout();
      transmitStatus();
      break;
    default:
      break;
  }

  // Process any received messages.
  NMEA2000.ParseMessages();
  // The above may have resulted in acquisition of a new source
  // address, so we check if there has been a change and if so save the
  // new address to EEPROM for future re-use.
  if (NMEA2000.ReadResetAddressChanged()) EEPROM.update(EEPROM_SOURCE_ADDRESS, NMEA2000.GetN2kSource());

  STATUS_LED_MANAGER.loop();
  STATE_LED_MANAGER.loop();
  
  #ifdef DEBUG_SERIAL
  debugDump();
  #endif
}

///////////////////////////////////////////////////////////////////////
// SWITCH INTERFACE OPERATIONS
///////////////////////////////////////////////////////////////////////

/**********************************************************************
 * Transmit a PGN126208 Command Group Function for PGN128776 Windlass
 * Control Status on <windlass> iff <windlass> has both a vaild windlass
 * identifier and a valid SOURCE_ADDRESS.
 * 
 * The outgoing message will always contain fields 02 WindlassIdentifier
 * and 03 WindlassDirectionControl of PGN128776 with additional fields
 * only being included if their data values have changed since last
 * transmission.
 */
void transmitWindlassControl(Windlass windlass) {
  if (windlass.getWindlassIdentifier() != INSTANCE_UNDEFINED) {
    if (windlass.SOURCE_ADDRESS != BROADCAST_SOURCE_ADDRESS) {
      tN2kMsg N2kMsg;
      N2kMsg.SetPGN(126208UL);                        // Command Group Function 
      N2kMsg.Priority = 2;                            // High priority
      N2kMsg.Destination = windlass.SOURCE_ADDRESS;   // Send direct to the configured thruster module
      N2kMsg.AddByte(0x01);                           // This is a command message
      N2kMsg.Add3ByteInt(128776UL);                   // Windlass Control Status PGN
      N2kMsg.AddByte(0xF8);                           // Retain existing priority

      // Count how many PGN128776 properties are dirty because these are
      // the ones we will be transmitting.
      int dirtyCount = 2; // We always transmit F02 ThrusterIdentifier and F03 ThrusterDirectionControl
      for (int i = 4; i <= PGN128776_FieldCount; i++) if (windlass.PGN128776v.isDirty(i)) dirtyCount++;
      N2kMsg.AddByte(dirtyCount);                     // Number of parameter pairs

      // Iterate over every PGN128776 property and if it's dirty, add
      // it to this message.
      for (int i = 2; i<= PGN128776_FieldCount; i++) {
        switch (i) {
          case 2: // Field 2 (WindlassIdentifier)
            N2kMsg.AddByte(i);                   
            N2kMsg.AddByte(windlass.PGN128776v.getThrusterIdentifier());
            break;
          case 3: // Field 3 (WindlassDirectionControl)
            N2kMsg.AddByte(i);                   
            N2kMsg.AddByte(windlass.PGN128776v.getWindlassDirectionControl());
            break;
          case 4: // Field 4 (SpeedControl)
            if (windlass.PGN128006v.isDirty(i)) {
              N2kMsg.AddByte(i);                 
              N2kMsg.AddByte(windlass.PGN128776v.getSpeedControl());
              windlass.PGN128776v.setClean(i);
            }
            break;
          case 5: // Field 5 (SpeedControlType)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);                   
              N2kMsg.AddByte(windlass.PGN128776v.getSpeedControlType());
              windlass.PGN128776v.setClean(i);
            }
            break;
          case 6: // Field 6 (AnchorDockingControl)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);                   
              N2kMsg.AddByte(windlass.PGN128776v.getSpeedControl());
              windlass.PGN128776v.setClean(i);
            }
            break;
          case 7: // Field 7 (PowerEnable)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);                   
              N2kMsg.AddByte(windlass.PGN128776v.getPowerEnable());
              PGN128776v.setClean(i);
            }
            break;
          case 8:// Field 8 (MechanicalLock)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);                   
              N2kMsg.AddByte(windlass.PGN128776v.getMechanicalLock());
              windlass.PGN128776v.setClean(i);
            }
            break;
          case 9: // Field 9 (DeckAndAnchorWash)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);                   
              N2kMsg.AddByte(windlass.PGN128776v.getDeckAndAnchorWash());
              windlass.PGN128776v.setClean(i);
            }
            break;
          case 10: // Field 10 (AnchorLight)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);                   
              N2kMsg.AddByte(windlass.PGN128776v.getAnchorLight());
              windlass.PGN128776v.setClean(i);
            }
            break;
          case 11: // Field 11 (CommandTimeout)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);                   
              N2kMsg.Add1ByteDouble(windlass.PGN128776v.getCommandTimeout(), 0.005);
              windlass.PGN128776v.setClean(i);
            }
            break;
          case 12: // Field 12 (WindlassControlEvents)
            if (windlass.PGN128776v.isDirty(i)) {
              N2kMsg.AddByte(i);
              N2kMsg.AddByte(windlass.getWindlassControlEvents().Events);
              windlass.PGN128776v.setClean(i);
            }
            break;
        }
      }
      NMEA2000.SendMsg(N2kMsg);
      LED_MANAGER.operate(GPIO_POWER_LED, 1, 1);
    }
  }
}

/**********************************************************************
 * Process all GPIO switch inputs supported by the module and
 * immediately take appropriate action. This function should be called
 * directly from loop: it will ensure that action is only taken at the
 * intervals defined by OPERATE_SWITCH_PROCESS_INTERVAL (for UP and DN
 * switch inputs) and PROGRAM_SWITCH_PROCESS_INTERVAL (for the PRG
 * switch input).
 * 
 * If UP or DN switch inputs for either module are being operated, then
 * a call is immediately made to transmitWindlassControl(). If the PRG
 * switch input is active, then the value set on the ADDRESS DIP switch
 * is saved to WINDLASS1 and the change persisted by writing the
 * windlass configuration to EEPROM.
 */ 
void processSwitchInputs() {
  static unsigned long deadlineW0 = 0UL;
  static unsigned long deadlineW1 = 0UL;
  static unsigned long deadlinePRG = 0UL;
  unsigned long now = millis();
  bool transmit;

  if ((WINDLASS0.PGN128776v.getCommandTimeout() != 0.0) && (now > deadlineW0)) {
    transmit = false;
    if (!DEBOUNCER.channelState(GPIO_W0_UP_SWITCH) && DEBOUNCER.channelState(GPIO_W0_DN_SWITCH)) {
      WINDLASS0.PGN128776v.setWindlassDirectionControl(N2kDD484_Up);
      transmit = true;
    }
    if (!DEBOUNCER.channelState(GPIO_W0_DN_SWITCH) && DEBOUNCER.channelState(GPIO_W0_UP_SWITCH)) {
      WINDLASS0.PGN128776v.setWindlassDirectionControl(N2kDD484_Down);
      transmit = true;
    }
    if (transmit) transmitWindlassControl(WINDLASS0);
    deadlineW0 = (now + OPERATE_SWITCH_PROCESS_INTERVAL));
  }
  if ((WINDLASS1.PGN128776v.getCommandTimeout() != 0.0) && (now > deadlineW1)) {
    transmit = false;
    if (!DEBOUNCER.channelState(GPIO_W1_UP_SWITCH) && DEBOUNCER.channelState(GPIO_W1_DN_SWITCH)) {
      WINDLASS1.PGN128776v.setWindlassDirectionControl(N2kDD484_Up);
      transmit = true;
    }
    if (!DEBOUNCER.channelState(GPIO_W1_DN_SWITCH) && DEBOUNCER.channelState(GPIO_W1_UP_SWITCH)) {
      WINDLASS1.PGN128776v.setWindlassDirectionControl(N2kDD484_Down);
      transmit = true;
    }
    if (transmit) transmitWindlassControl(WINDLASS1);
    deadlineW1 = (now + OPERATE_SWITCH_PROCESS_INTERVAL);
  }
  if ((!DEBOUNCER.channelState(GPIO_PROGRAM_SWITCH)) && (now > deadlinePRG)) {
    DILSWITCH.sample();
    WINDLASS1.setWindlassIdentifier(DILSWITCH.value());
    WINDLASS1.saveToEEPROM(EEPROM_WINDLASS1);
    deadlinePRG = (now + PROGRAM_SWITCH_PROCESS_INTERVAL);
  }
}

/**********************************************************************
 * Operates the UP and DOWN LED relays associated with the specified
 * <windlass> in responce to the value of the windlass state property.
 */
void operateIndicators(Windlass windlass, int up, int dn) {
  switch (windlass->PGN128777.getWindlassMotionStatus()) {
    case N2kDD480_RetrievalOccurring:
      LED_MANAGER.operate(up, 0, -1);
      break;
    case N2kDD480_DeploymentOccurring:
      LED_MANAGER.operate(dn, 0, -1);
      break;
    default:
      switch (windlass->PGN128777.getAnchorDockingStatus()) {
        case N2kDD482_FullyDocked:
          LEDMANAGER.operate(up, 1, 0);
          break;
        default:
          break;
      }
      break;
  }
}

/**********************************************************************
 * Handle any received PGN128776 Windlass Control Status messages
 * received from a remote windlass. This function maintains the N2K
 * source address of the remote windlasses configured by WINDLASS0 and
 * WINDLASS1.
 * 
 * A remote windlass configuration is only valid if the user
 * has configured a valid windlass identifier and if this is the case
 * the source address of any PGN128776 message received from the
 * identified remote device is saved to the windlass configuration
 * along with the time of receipt.
 * 
 * This function works in concert with checkConnection() to ensure that
 * each windlass configuration SOURCE_ADDRESS remains valid only when
 * the presence of an identified windlass can be confirmed on the N2K
 * bus.
 */  
void PGN128776Handler(const tN2kMsg &N2kMsg) {
  uint8_t SID;
  uint8_t WindlassIdentifier;
  tN2kDD484 WindlassDirectionControl;
  uint8_t SpeedControl;
  tN2kDD488 SpeedControlType;
  tN2kDD002 AnchorDockingControl;
  tN2kDD002 PowerEnable;
  tN2kDD002 MechanicalLock;
  tN2kDD002 DeckAndAnchorWash;
  tN2kDD002 AnchorLight;
  double CommandTimeout;
  tN2kDD478 WindlassControlEvents;

  if (parseN2kPGN128776(N2kMsg, SID, WindlassIdentifier, WindlassDirectionControl, SpeedControl, SpeedControlType,  AnchorDockingControl, PowerEnable, MechanicalLock, DeckAndAnchorWash, AnchorLight, CommandTimeout, WindlassControlEvents)) {
    if (WindlassIdentifier == WINDLASS0.PGN128776v.getWindlassIdentifier()) {
      WINDLASS0.SOURCE_ADDRESS = (unsigned char) N2kMsg.Source;
      WINDLASS0.SOURCE_ADDRESS_UPDATE_TIMESTAMP = millis();
      LED_MANAGER.operate(GPIO_POWER_LED, 1, 0);
    }
    if (WindlassIdentifier == WINDLASS1.PGN128776v.getWindlassIdentifier()) {
      WINDLASS1.SOURCE_ADDRESS = (unsigned char) N2kMsg.Source;
      WINDLASS1.SOURCE_ADDRESS_UPDATE_TIMESTAMP = millis();
      LED_MANAGER.operate(GPIO_POWER_LED, 1, 0);
    }
  }
}

/**********************************************************************
 * Handle incoming PGN128777 messages and, iff they relate to one of
 * the windlasses we control, update our local PGN. The data received
 * is used by operateIndicators() to modulate the indicator outputs in
 * a way that can be used to build an informative UI.
 */
void PGN12877Handler(const tN2kMsg &N2kMsg) {
  uint8_t SID;
  uint8_t WindlassIdentifier;
  double RodeCounterValue;
  double WindlassLineSpeed;
  tN2kDD480 WindlassMotionStatus;
  tN2kDD481 RodeTypeStatus;
  tN2kDD482 AnchorDockingStatus;
  tN2kDD483 WindlassOperatingEvents;

  if (parseN2kPGN128777(N2kMsg, SID, WindlassIdentifier, RodeCounterValue, WindlassLineSpeed, WindlassMotionStatus, RodeTypeStatus, AnchorDockingStatus, WindlassOperatingEvents)) {
    if (WindlassIdentifier = WINDLASS0.PGN128777v.getWindlassIdentifier()) {
      WINDLASS0.PGN128777v.setRodeCounterValue(RodeCounterValue);
      WINDLASS0.PGN128777v.setWindlassLineSpeed(WindlassLineSpeed);
      WINDLASS0.PGN128777v.setWindlassMotionStatus(WindlassMotionStatus);
      WINDLASS0.PGN128777v.setRodeTypeStatus(RodeTypeStatus);
      WINDLASS0.PGN128777v.setAnchorDockingStatus(AnchorDockingStatus);
      WINDLASS0.PGN128777v.setWindlassOperatingEvents(WindlassOperatingEvents);
    }
  }

  if (parseN2kPGN128777(N2kMsg, SID, WindlassIdentifier, RodeCounterValue, WindlassLineSpeed, WindlassMotionStatus, RodeTypeStatus, AnchorDockingStatus, WindlassOperatingEvents)) {
    if (WindlassIdentifier = WINDLASS1.PGN128777v.getWindlassIdentifier()) {
      WINDLASS1.PGN128777v.setRodeCounterValue(RodeCounterValue);
      WINDLASS1.PGN128777v.setWindlassLineSpeed(WindlassLineSpeed);
      WINDLASS1.PGN128777v.setWindlassMotionStatus(WindlassMotionStatus);
      WINDLASS1.PGN128777v.setRodeTypeStatus(RodeTypeStatus);
      WINDLASS1.PGN128777v.setAnchorDockingStatus(AnchorDockingStatus);
      WINDLASS1.PGN128777v.setWindlassOperatingEvents(WindlassOperatingEvents);
    }
  }
}

/**********************************************************************
 * Handle incoming PGN128778 messages and, iff they relate to one of
 * the windlasses we control, update our local PGN. The data received
 * is used by operateIndicators() to modulate the indicator outputs in
 * a way that can be used to build an informative UI.
 */
void PGN128778Handler(const tN2kMsg &N2kMsg) {
  uint8_t SID;
  uint8_t WindlassIdentifier;
  double TotalMotorTime;
  double ControllerVoltage;
  double MotorCurrent;
  tN2kDD477 WindlassMonitoringEvents;

  if (parseN2kPGN128778(N2kMsg, SID, WindlassIdentifier, TotalMotorTime, ControllerVoltage, MotorCurrent, WindlassMonitoringEvents)) {
    if (WindlassIdentifier = WINDLASS0.PGN128778v.getWindlassIdentifier()) {
      WINDLASS0.PGN128778v.setTotalMotorTime(TotalMotorTime);
      WINDLASS0.PGN128777v.setControllerVoltage(ControllerVoltage);
      WINDLASS0.PGN128777v.setMotorCurrent(MotorCurrent);
      WINDLASS0.PGN128777v.setWindlassMonitoringEvents(WindlassMonitoringEvents);
    }
  }

  if (parseN2kPGN128778(N2kMsg, SID, WindlassIdentifier, TotalMotorTime, ControllerVoltage, MotorCurrent, WindlassMonitoringEvents)) {
    if (WindlassIdentifier = WINDLASS1.PGN128777v.getWindlassIdentifier()) {
      WINDLASS1.PGN128778v.setTotalMotorTime(TotalMotorTime);
      WINDLASS1.PGN128777v.setControllerVoltage(ControllerVoltage);
      WINDLASS1.PGN128777v.setMotorCurrent(MotorCurrent);
      WINDLASS1.PGN128777v.setWindlassMonitoringEvents(WindlassMonitoringEvents);
    }
  }
}

/**********************************************************************
 * checkConnection() examines the SOURCE_ADDRESS_UPDATE_TIMESTAMP of
 * each windlass and if the timestamp is older than
 * WINDLASS_SOURCE_ADDRESS_TIMEOUT it sets the windlass' SOURCE_ADDRESS
 * to the broadcast source address, effectively disabling control of
 * the windlass. The function works in concert withPGN128776Handler()
 * which seeks to maintain a valid source address for windlasses that
 * are active on the N2K bus.
 * 
 * The function should be called directly from loop() and it will only
 * perform its task every REMOTE_WINDLASS_CHECK_INTERVAL.
 *   It tries to
 * make sure that a remote windlass connection is still viable by
 * checking that PGN128776Handler() has received status updates from
 * the remote within the period defined by THRUSTER_SOURCE_ADDRESS_TIMEOUT.
 * If the status updates have dried up then THRUSTER_SOURCE_ADDRESS is
 * reset to the broadcast address and the power LED is returned to
 * continuous flashing to indicate no connection.
 */
void checkConnection() {
  static unsigned long deadline = 0UL;
  unsigned long now = millis();

  if (now > deadline) {
    if (WINDLASS0.getWindlassIdentifier() != INSTANCE_UNDEFINED) {
      if (now > (WINDLASS0.SOURCE_ADDRESS_UPDATE_TIMESTAMP + REMOTE_WINDLASS_CHECK_INTERVAL)) {
        WINDLASS0.SOURCE_ADDRESS = BROADCAST_SOURCE_ADDRESS;
        LED_MANAGER.operate(GPIO_POWER_LED, 0, 10000);
      }
    }
    if (WINDLASS1.getWindlassIdentifier() != INSTANCE_UNDEFINED) {
      if (now > (WINDLASS1.SOURCE_ADDRESS_UPDATE_TIMESTAMP + REMOTE_WINDLASS_CHECK_INTERVAL)) {
        WINDLASS1.SOURCE_ADDRESS = BROADCAST_SOURCE_ADDRESS;
        LED_MANAGER.operate(GPIO_POWER_LED, 0, 10000);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////
// START OF RELAY INTERFACE OPERATIONS
///////////////////////////////////////////////////////////////////////

/**********************************************************************
 * Processes sensor inputs every SWITCH_PROCESS_INTERVAL milliseconds.
 * Must be called directly from loop(). The defined time interval must
 * be less than the rotation period of the windlass being monitored.
 */

void processSensorInputs() {
  static bool rotationSensorProcessed = false;
  static unsigned long deadline = 0UL;
  unsigned long now = millis();
  if (now > deadline) {  
    switch (!DEBOUNCER.channelState(GPIO_SENSOR_ROT)) {
      case true:
        if (!rotationSensorProcessed) {
          spudpole.bumpRotationCount();
          rotationSensorProcessed = true;
        }
        break;
      case false:
        rotationSensorProcessed = false;
        break;
    }
    // Docked sensor
    WINDLASS0.PGN128777v.setAnchorDockingStatus((!DEBOUNCER.channelState(GPIO_W0_DOCKED_SENSOR))?N2kDD482_FullyDocked:N2kDD482_NotDocked);
    // Deployed sensor
    WINDLASS0.PGN128777v.setAnchorDockingStatus((!DEBOUNCER.channelState(GPIO_W0_DEPLOYED_SENSOR))?N2kDD482_// WHAT TO DO HERE
    // Motion sensors
    if ((!DEBOUNCER.channelState(GPIO_W0_DEPLOYING_SENSOR)) || (!DEBOUNCER.channelState(GPIO_W0_DEPLOYING_SENSOR))) {
      if (!DEBOUNCER.channelState(GPIO_W0_DEPLOYING_SENSOR)) WINDLASS0.PGN128777v.setWindlassMotionStatus(?N2kDD480_DeploymentOccurring);
      if (!DEBOUNCER.channelState(GPIO_W0_RETRIEVING_SENSOR)) WINDLASS0.PGN128777v.setWindlassMotionStatus(?N2kDD480_RetrievalOccurring);
    } else {
      WINDLASS0.PGN128777v.setWindlassMotionStatus(N2kDD480_WindlassStopped);
    }
    // Stress sensor
    if (!DEBOUNCER.channelState(GPIO_W0_STRESS_SENSOR)) {
      WINDLASS0.PGN128778v.setWindlassMonitoringEvents(WINDLASS0.PGN128778v.getWindlassMonitoringEvents().ControllerOverCurrentCutout = 1);
    } else {
      WINDLASS0.PGN128778v.setWindlassMonitoringEvents(WINDLASS0.PGN128778v.getWindlassMonitoringEvents().ControllerOverCurrentCutout = 0);
    } 
    
     Spudpole::YES:Spudpole::NO);
    spudpole.setOperatingState((!DEBOUNCER.channelState(GPIO_SENSOR_DPG))?Windlass::DEPLOYING:Windlass::STOPPED);
    spudpole.setOperatingState((!DEBOUNCER.channelState(GPIO_SENSOR_RTG))?Windlass::RETRIEVING:Windlass::STOPPED);
    deadline = (now + SENSOR_PROCESS_INTERVAL);
  }
}

void onPulse() {
  static unsigned long lastPulseTime= 0UL;
  unsigned long now = millis();

  WINDLASS0.PGN128777v.setRodeCounterValue(WINDLASS0.PGN128777v.getRodeCounterValue() + PULSE_DISTANCE);
  WINDLASS0.PGN128777v.setWindlassLineSpeed(PULSE_DISTANCE / ((now - lastPulseTime) / 1000));
}


/**********************************************************************
 * Sets the state of the output relays from the current state of
 * <windlass>'s PGN 128776 property. Makes a note of the command
 * timeout so that checkTimeout() can cancel things if control messages
 * dry up and also maintains motor run-time data.
 */
void setRelays(Windlass windlass, int gpioUp, int gpioDn) {
  switch (windlass.PGN128776v.getWindlassOirectionControl()) {
    case N2kDD484_Up:
      digitalWrite(gpioUp, 1);
      digitalWrite(gpioDn, 0);
      windlass.OPERATING_TIMEOUT = (millis() + (windlass.PGN128776v.getCommandTimeout() * 1000));
      if (windlass.START_TIME == 0UL) windlass.START_TIME = millis();
      break;
    case N2kDD484_Down:
      digitalWrite(gpioUp, 0);
      digitalWrite(gpioDn, 1);
      windlass.OPERATING_TIMEOUT = (millis() + (windlass.PGN128776v.getCommandTimeout() * 1000));
      if (windlass.START_TIME == 0UL) windlass.START_TIME = millis();
      break;
    default:
      digitalWrite(gpioUp, 0);
      digitalWrite(gpioDn, 0);
      windlass.OPERATING_TIMEOUT = 0UL;
      // Set odometer and save it here.
      break;
  }
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
void checkTimeout() {
  unsigned long now = millis();

  if ((WINDLASS0.OPERATING_TIMEOUT != 0) && (now > WINDLASS0.OPERATING_TIMEOUT)) {
    WINDLASS0.PGN128776v.setWindlassDirectionControl(N2kDD484_Off);
  }
  if ((WINDLASS1.OPERATING_TIMEOUT != 0) && (now > WINDLASS1.OPERATING_TIMEOUT)) {
    WINDLASS1.PGN128776v.setWindlassDirectionControl(N2kDD484_Off);
  }
}

/**********************************************************************
 * Transmit all the windlass PGNs at the intervals determined by the
 * current operating state.
 */
void transmitStatus() {
  unsigned char SID = 0;

  static unsigned long PGN128776Deadline = 0UL;
  static unsigned long PGN128777Deadline = 0UL;
  static unsigned long PGN128778Deadline = 0UL;
  unsigned long now = millis();

  if ((PGN128776_UPDATE_INTERVAL != 0UL) && (now > PGN128776Deadline)) {
    transmitPGN128776(SID);
    PGN128776Deadline = (now + isOperating()?PGN128776_DynamicUpdateInterval:PGN128776_StaticUpdateInterval);
  }

  if ((PGN128777_UPDATE_INTERVAL != 0UL) && (now > PGN128777Deadline)) {
    transmitPGN128777(SID);
    PGN128007Deadline = (now + isOperating()?PGN128777_DynamicUpdateInterval:PGN128777_StaticUpdateInterval);
  }

  if ((PGN128778_UPDATE_INTERVAL != 0UL) && (now > PGN128778Deadline)) {
    transmitPGN128778(SID);
    PGN128778Deadline = (now + isOperating()?PGN128778_DynamicUpdateInterval:PGN128778_StaticUpdateInterval);
  }

  SID++;
}

/**********************************************************************
 * Transmit PGN128776.
 */
void transmitPGN128776(uint8_t sid, Windlass windlass) {
  #ifdef DEBUG_SERIAL
  Serial.println("TRANSMITTING PGN128776");
  #endif
  tN2kMsg N2kMsg;
  SetN2kPGN128776(
    N2kMsg,
    sid,
    windlass.PGN128776v.getWindlassIdentifier(),
    windlass.PGN128006v.getWindlassDirectionControl(),
    windlass.PGN128006v.getSpeedControl(),
    windlass.PGN128006v.getSpeedControlType(),
    windlass.PGN128776v.getAnchorDockingControl(),
    windlass.PGN128776v.getPowerEnable(),
    windlass.PGN128776v.getMechanicalLock(),
    windlass.PGN128776v.getDeckAndAnchorWash(),
    windlass.PGN128776v.getAnchorLight(),
    windlass.PGN128776v.getCommandTimeout(),
    windlass.PGN128776v.getWindlassControlEvents()
  );
  NMEA2000.SendMsg(N2kMsg);
}

/**********************************************************************
 * Transmit PGN128777.
 */
void transmitPGN128777(uint8_t sid, Windlass windlass) {
  #ifdef DEBUG_SERIAL
  Serial.println("TRANSMITTING PGN128777");
  #endif
  tN2kMsg N2kMsg;
  SetN2kPGN128777(
    N2kMsg,
    sid,
    windlass.PGN128777v.getWindlassIdentifier(),
    windlass.PGN128777v.getRodeCounterValue(),
    windlass.PGN128777v.getWindlassLineSpeed(),
    windlass.PGN128777v.getWindlassMotionStatus(),
    windlass.PGN128777v.getRodeTypeStatus(),
    windlass.PGN128777v.getAnchorDockingStatus(),
    windlass.PGN128777v.getWindlassOperatingEvents()
  );
  NMEA2000.SendMsg(N2kMsg);
}

/**********************************************************************
 * Transmit PGN128778.
 */
void transmitPGN128778(uint8_t sid, Windlass windlass) {
  #ifdef DEBUG_SERIAL
  Serial.println("TRANSMITTING PGN128778");
  #endif
  tN2kMsg N2kMsg;
  SetN2kPGN128778(
    N2kMsg,
    sid,
    windlass.PGN128777v.getWindlassIdentifier(),
    windlass.PGN128777v.getTotalMotorTime(),
    windlass.PGN128777v.getControllerVoltage(),
    windlass.PGN128777v.getMotorCurrent(),
    windlass.PGN128777v.getWindlassMonitoringEvents()
  );
  NMEA2000.SendMsg(N2kMsg);
}

/**********************************************************************
 * Some MFDs issue PGN 059904 ISO Request PGNs in order to get data
 * from a device, so we might as well pander to them.
 */
bool ISORequestHandler(unsigned long RequestedPGN, unsigned char Requester, int DeviceIndex) {
  if (SELECTED_OPERATING_MODE == RELAY_INTERFACE) {
    switch (RequestedPGN) {
      case 128776UL: transmitPGN128776(0); return(true); break;
      case 128777UL: transmitPGN128777(0); return(true); break;
      case 128778UL: transmitPGN128778(0); return(true); break;
      default: break;
    }
  }
  return(false);
}

/**********************************************************************
 * updatePGN128776() is used as a callback by handlers in the
 * GroupFunctionHandlerForPGN128776 class. It accepts a sparse array
 * of values for properties in the eponymous PGN and, if a value is
 * marked as modified, it updates the local property with the supplied
 * value iff the ThrusterIdentifier in the update array matches the
 * identifier defined for this module.
 */
void updatePGN128776(PGN128776_Field fields[]) {
  bool canUpdate = false;
  
  if ((fields[2].dirty) && (fields[2] == WINDLASS0.PGN128776v.getWindlassIdentifier())) {
    for (int i = 3; i <= PGN128776_FieldCount; i++) {
      if (fields[i].dirty) WINDLASS0.PGN128776v.setProperty(i, fields[i].value);
    }
  }
}

///////////////////////////////////////////////////////////////////////
// END OF DEVICE MODE SPECIALISATION
///////////////////////////////////////////////////////////////////////

void messageHandler(const tN2kMsg &N2kMsg) {
  int iHandler;
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}



#ifdef DEBUG_SERIAL
void debugDump() {
  static unsigned long deadline = 0UL;
  unsigned long now = millis();
  if (now > deadline) {
    Serial.print("DEBUG DUMP @ "); Serial.println(now);
    Serial.print("W0 instance:  "); Serial.println(WINDLASS0.instance, HEX);
    Serial.print("W0 address:   "); Serial.println(WINDLASS0.address, HEX);
    Serial.print("W0 UP switch: "); Serial.println(WINDLASS0.pDebouncer->channelState(WINDLASS0.upSwitchGPIO));
    Serial.print("W0 DN switch: "); Serial.println(WINDLASS0.pDebouncer->channelState(WINDLASS0.downSwitchGPIO));
    deadline = (now + DEBUG_SERIAL_INTERVAL);
  }
}
#endif
