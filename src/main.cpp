
//------------------------------- Board Check ----------------------------------


/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>              //for ESP8266 use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver

#include <ArduinoUAVCAN.h>
#include <ArduinoMCP2515.h>

#include <OneWire.h>
#include <DallasTemperature.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

#define UC_NAME "pl.simle.r5.temp_i2c"

static const CanardNodeID UC_ID = 17;
static const uavcan_node_GetInfo_Response_1_0 GET_INFO_DATA = {
    /// uavcan.node.Version.1.0 protocol_version
    {1, 0},
    /// uavcan.node.Version.1.0 hardware_version
    {1, 0},
    /// uavcan.node.Version.1.0 software_version
    {0, 1},
    /// saturated uint64 software_vcs_revision_id
    NULL,
    /// saturated uint8[16] unique_id
    {0x86, 0xcc, 0xed, 0x61, 0x97, 0x1f, 0x4a, 0xf9, 
     0x9d, 0x19, 0x51, 0xc3, 0x9a, 0xb9, 0xa8, 0xd0},
    /// saturated uint8[<=50] name
    {
        UC_NAME,
        strlen(UC_NAME)
    },
};

static const uint32_t DESIRED_BIT_RATE = 125UL * 1000UL; // 1 kb/s
// static const int LED_BUILTIN = 2;

static int const MKRCAN_MCP2515_CS_PIN  = A1;
static int const MKRCAN_MCP2515_INT_PIN = A2;

static int const TEMP_ONE_WIRE_PIN = A3;

static CanardPortID const TEMP_A_PORT_ID   = 2137U;
static CanardPortID const TEMP_B_PORT_ID   = 2138U;

// external
static CanardPortID const LOADCELL_PORT_ID   = 1337U;
static CanardPortID const DHT_T_PORT_ID   = 1338U;
static CanardPortID const DHT_H_PORT_ID   = 1339U;

static int const TEMPERATURE_PRECISION = 12;



/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void    spi_select        ();
void    spi_deselect       ();
uint8_t spi_transfer       (uint8_t const);
void    onExternalEvent    ();
bool transmitCanFrame(CanardFrame const &);
void    onReceiveBufferFull(CanardFrame const &);

void onGetInfo_1_0_Request_Received(CanardTransfer const &, ArduinoUAVCAN &);
void onLoadcell_1_0_Received(CanardTransfer const &, ArduinoUAVCAN &);
void onTempOutside_1_0_Received(CanardTransfer const &, ArduinoUAVCAN &);

void on_i2c_request();

void get_temp();

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515(spi_select,
                       spi_deselect,
                       spi_transfer,
                       micros,
                       onReceiveBufferFull,
                       nullptr);

ArduinoUAVCAN uc(UC_ID, transmitCanFrame);

Heartbeat_1_0<> hb;
Real32_1_0<LOADCELL_PORT_ID> scale_measurment;
Real32_1_0<DHT_T_PORT_ID> tempOutside;
//TODO: refactor to list
Real32_1_0<TEMP_A_PORT_ID> tempAbove;
Real32_1_0<TEMP_B_PORT_ID> tempBelow;

static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;

OneWire oneWire(TEMP_ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress thermoAbove, thermoBelow;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  int error;
  
 
  Wire.begin(8);
  Wire.onRequest(on_i2c_request);
  //--- Switch on builtin led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //--- Start serial
  Serial.begin(115200);
  // while (!Serial)
  // {
  // }
  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_125kBPS_8MHZ);
  mcp2515.setNormalMode();

    // locate devices on the bus
  Serial.print("Locating devices...");
   delay(4000);
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

    oneWire.reset_search();
  if (!oneWire.search(thermoAbove)) Serial.println("Unable to find address for Device 0"); 
  if (!oneWire.search(thermoBelow)) Serial.println("Unable to find address for Device 1");

  Serial.print("Device 0 Address: ");
  printAddress(thermoAbove);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(thermoBelow);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(thermoAbove, TEMPERATURE_PRECISION);
  sensors.setResolution(thermoBelow, TEMPERATURE_PRECISION);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(thermoAbove), DEC); 
  Serial.println();

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  uc.subscribe<GetInfo_1_0::Request<>>(onGetInfo_1_0_Request_Received);
  uc.subscribe<Real32_1_0<LOADCELL_PORT_ID>>(onLoadcell_1_0_Received);
  uc.subscribe<Real32_1_0<DHT_T_PORT_ID>>(onTempOutside_1_0_Received);
}

void loop()
{
  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0<>::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if (now - prev >= 1000)
  {
    get_temp();
    uc.publish(hb);
    uc.publish(tempAbove);
    prev = now;

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.print(now);
    Serial.print(" ");
    Serial.print(UC_NAME);
    Serial.print(" Sent: ");
    Serial.print(gSentFrameCount);
    Serial.print("\t");
    Serial.print("Receive: ");
    Serial.print(gReceivedFrameCount);
    Serial.print("\t");

  }
    /* Transmit all enqeued CAN frames */
  while (uc.transmitCanFrame())
  {
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void on_i2c_request() {
  Serial.print(".");
  float vec[4] = {
    (float) tempAbove.data.value, 
    (float) tempBelow.data.value,
    (float) tempOutside.data.value,
    (float) scale_measurment.data.value
  };
  Wire.write((uint8_t*) vec, sizeof(vec));
}

// function to print the temperature for a device
void get_temp()
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempA = sensors.getTempC(thermoAbove);
  if(tempA == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  tempAbove.data.value = tempA;
  float tempB = sensors.getTempC(thermoBelow);
  if(tempB == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  tempBelow.data.value = tempB;
  printAddress(thermoAbove);
  Serial.print(" ");
  Serial.println(tempA);
  printAddress(thermoBelow);
  Serial.print(" ");
  Serial.println(tempB);
}

void spi_select()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

void onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

bool transmitCanFrame(CanardFrame const & frame)
{
  gSentFrameCount += 1;
  return mcp2515.transmit(frame);
}

void onReceiveBufferFull(CanardFrame const & frame)
{
  gReceivedFrameCount += 1;
  uc.onCanFrameReceived(frame);
}

void onLoadcell_1_0_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uc) {
  Real32_1_0<LOADCELL_PORT_ID> const d = Real32_1_0<LOADCELL_PORT_ID>::deserialize(transfer);
  scale_measurment = d;
  Serial.println(scale_measurment.data.value);
}

void onTempOutside_1_0_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uc) {
  Real32_1_0<DHT_T_PORT_ID> const d = Real32_1_0<DHT_T_PORT_ID>::deserialize(transfer);
  tempOutside = d;
  Serial.println(tempOutside.data.value);
}

void onGetInfo_1_0_Request_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uc)
{
  GetInfo_1_0::Response<> rsp = GetInfo_1_0::Response<>();
  rsp.data = GET_INFO_DATA;
  Serial.println("onGetInfo_1_0_Request_Received");
  uc.respond(rsp, transfer.remote_node_id, transfer.transfer_id);
}

