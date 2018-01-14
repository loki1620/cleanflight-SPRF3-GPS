/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * FlySky iBus telemetry implementation by CraigJPerry.
 * Unit tests and some additions by Unitware
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
// #include <string.h>

#include "platform.h"
//#include "common/utils.h"
#include "telemetry/telemetry.h"
#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"

static uint16_t calculateChecksum(const uint8_t *ibusPacket);


#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "sensors/battery.h"
#include "fc/rc_controls.h"
#include "fc/config.h"
#include "sensors/gyro.h"
#include "drivers/accgyro/accgyro.h"
#include "fc/runtime_config.h"
#include "sensors/acceleration.h"
#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "flight/imu.h"
#include "flight/altitude.h"
#include "flight/navigation.h"
#include "io/gps.h"


#define IBUS_TEMPERATURE_OFFSET  (400)
#define INVALID_IBUS_ADDRESS 0
#define IBUS_BUFFSIZE 33				// biggest iBus message seen so far + 1
#define IBUS_HEADER_FOOTER_SIZE 4
#define TEM_1(x) ((400+(x))&0xFF)
#define TEM_2(x) ((400+(x))>>8)
#define LBYTE(x) ((x)&0xFF)
#define HBYTE(x) ((x)>>8&0xFF)

#define IBUS_2BYTE_SESNSOR 2
#define IBUS_4BYTE_SESNSOR 4

typedef uint8_t ibusAddress_t;

typedef enum {
    IBUS_COMMAND_DISCOVER_SENSOR      = 0x80,
    IBUS_COMMAND_SENSOR_TYPE          = 0x90,
    IBUS_COMMAND_MEASUREMENT          = 0xA0
} ibusCommand_e;

const uint8_t FULL_GPS_IDS[] = {
	IBUS_SENSOR_TYPE_GPS_STATUS,
  IBUS_SENSOR_TYPE_GPS_LAT,
  IBUS_SENSOR_TYPE_GPS_LON,
  IBUS_SENSOR_TYPE_GPS_ALT,
};

const uint8_t FULL_VOLT_IDS[] = {
	IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,
	IBUS_SENSOR_TYPE_CELL,
	IBUS_SENSOR_TYPE_BAT_CURR,
	IBUS_SENSOR_TYPE_FUEL,
	IBUS_SENSOR_TYPE_RPM,
};

const uint8_t FULL_ACC_IDS[] = {
	IBUS_SENSOR_TYPE_ACC_X,
	IBUS_SENSOR_TYPE_ACC_Y,
	IBUS_SENSOR_TYPE_ACC_Z,
  IBUS_SENSOR_TYPE_ROLL,
  IBUS_SENSOR_TYPE_PITCH,
  IBUS_SENSOR_TYPE_YAW,
};


/* Address lookup relative to the sensor base address which is the lowest address seen by the FC
   The actual lowest value is likely to change when sensors are daisy chained */

static uint8_t sensorAddressTypeLookup[15];

static serialPort_t *ibusSerialPort = NULL;
static ibusAddress_t ibusBaseAddress = INVALID_IBUS_ADDRESS;
static uint8_t sendBuffer[IBUS_BUFFSIZE];


static uint8_t getSensorID(ibusAddress_t address){
  if(ibusBaseAddress == INVALID_IBUS_ADDRESS) return IBUS_SENSOR_TYPE_UNKNOWN;
  return sensorAddressTypeLookup[address - ibusBaseAddress];
}

static uint8_t getSensorLength(uint8_t sensorID){
  uint8_t size = 0;
  uint8_t i = 0;
  if(sensorID == IBUS_SENSOR_TYPE_GPS_FULL){
    for (i = 0; i < sizeof(FULL_GPS_IDS); i++) {
      size += getSensorLength(FULL_GPS_IDS[i]);
    }
    return size;
  }
  if(sensorID == IBUS_SENSOR_TYPE_VOLT_FULL){
    for (i = 0; i < sizeof(FULL_VOLT_IDS); i++) {
      size += getSensorLength(FULL_VOLT_IDS[i]);
    }
    return size;
  }
  if(sensorID == IBUS_SENSOR_TYPE_ACC_FULL){
    for (i = 0; i < sizeof(FULL_ACC_IDS); i++) {
      size += getSensorLength(FULL_ACC_IDS[i]);
    }
    return size;
  }
  if(sensorID >= IBUS_SENSOR_TYPE_TEMPERATURE && sensorID <= IBUS_SENSOR_TYPE_FLIGHT_MODE)  return IBUS_2BYTE_SESNSOR;
  if(sensorID >= IBUS_SENSOR_TYPE_ODO1 && sensorID <= IBUS_SENSOR_TYPE_SPE)  return IBUS_2BYTE_SESNSOR;
  if(sensorID == IBUS_SENSOR_TYPE_ALT_FLYSKY) return IBUS_2BYTE_SESNSOR;
  if(sensorID >= IBUS_SENSOR_TYPE_GPS_LAT && sensorID <= IBUS_SENSOR_TYPE_ALT_MAX) return IBUS_4BYTE_SESNSOR;
  if(sensorID == IBUS_SENSOR_TYPE_PRES) return IBUS_4BYTE_SESNSOR;
  return IBUS_2BYTE_SESNSOR;
}

static uint8_t transmitIbusPacket()
{
    if(sendBuffer[0]==INVALID_IBUS_ADDRESS) return 0;
    uint8_t frameLength = sendBuffer[0];
    uint8_t payloadLength = frameLength - IBUS_CHECKSUM_SIZE;
    uint16_t checksum = calculateChecksum(sendBuffer);
    for (size_t i = 0; i < payloadLength; i++) {
        serialWrite(ibusSerialPort, sendBuffer[i]);
    }
    serialWrite(ibusSerialPort, checksum & 0xFF);
    serialWrite(ibusSerialPort, checksum >> 8);
    return frameLength;
}

static void setIbusDiscoverSensorReply(ibusAddress_t address)
{
    sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE;
    sendBuffer[1] = IBUS_COMMAND_DISCOVER_SENSOR | address;
}

static void setIbusSensorType(ibusAddress_t address)
{
    uint8_t sensorID = getSensorID(address);
    uint8_t sensorLength = getSensorLength(sensorID);
    sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE + 2;
    sendBuffer[1] = IBUS_COMMAND_SENSOR_TYPE | address;
    sendBuffer[2] = sensorID;
    sendBuffer[3] = sensorLength;
}

static void set16(uint8_t* buffer, int16_t measurement)
{
  buffer[0] = LBYTE(measurement);
  buffer[1] = HBYTE(measurement);
}
static void set16u(uint8_t* buffer, uint16_t measurement)
{
  buffer[0] = LBYTE(measurement);
  buffer[1] = HBYTE(measurement);
}
static void set32(uint8_t* buffer, int32_t val) {
  buffer[0] = (uint8_t)val;
  buffer[1] = (uint8_t)(((uint32_t)val >> 8) & 0xFF);
  buffer[2] = (uint8_t)(((uint32_t)val >> 16) & 0xFF);
  buffer[3] = (uint8_t)(((uint32_t)val >> 24) & 0xFF);
}
static void set32u(uint8_t* buffer, uint32_t val) {
    buffer[0] = (uint8_t)val;
    buffer[1] = (uint8_t)(((uint32_t)val >> 8) & 0xFF);
    buffer[2] = (uint8_t)(((uint32_t)val >> 16) & 0xFF);
    buffer[3] = (uint8_t)(((uint32_t)val >> 24) & 0xFF);
}

static void setVoltage(uint8_t* buffer){
  uint16_t voltage = getBatteryVoltage() *10;
  if (telemetryConfig()->report_cell_voltage) {
    voltage /= getBatteryCellCount();
  }
  set16u(buffer, voltage);
}

static int32_t getTemperature() {
  int32_t temperature = 0;
  if (sensors(SENSOR_BARO)) temperature = (uint16_t) ((baro.baroTemperature + 50) / 10);
  else temperature = gyroGetTemperature() * 10;
  return temperature;
}
static void setTemperature(uint8_t* buffer){
    set16u(buffer, (uint16_t)(getTemperature() + IBUS_TEMPERATURE_OFFSET));
}
static void setCellVoltage(uint8_t* buffer){
  uint16_t voltage = getBatteryAverageCellVoltage() *10;
  set16u(buffer, voltage);
}
static void setBatCurrent(uint8_t* buffer)
{
  uint16_t current = (uint16_t)getAmperage();
  set16u(buffer, current);
}
static void setFuel(uint8_t* buffer){
  if (batteryConfig()->batteryCapacity > 0) {
      //calculateBatteryPercentageRemaining()
      //float value = constrain(batteryConfig()->batteryCapacity - getMAhDrawn(), 0, batteryConfig()->batteryCapacity);
      set16u(buffer, (uint16_t)calculateBatteryPercentageRemaining());

  } else {
      set16u(buffer, (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF));
  }
}

static void setRPM(uint8_t* buffer){
  if (ARMING_FLAG(ARMED)) {
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    uint16_t throttleForRPM = rcCommand[THROTTLE];  // / BLADE_NUMBER_DIVIDER;
    if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP)) throttleForRPM = 0;
    set16u(buffer, throttleForRPM);
  } else {
    set16u(buffer, (uint16_t)(batteryConfig()->batteryCapacity));//  / BLADE_NUMBER_DIVIDER
  }
}

static void setVario(uint8_t* buffer){
  int32_t vario = getEstimatedVario(); //cm/s
  set16(buffer, (int16_t)vario);
}

static void setGPSstatus(uint8_t* buffer){
  uint8_t gpsFixType = 0;
  uint8_t sats = 0;
  #if defined(GPS)
  if (sensors(SENSOR_GPS)) {
      if (!STATE(GPS_FIX)) gpsFixType = 1;
      else gpsFixType = gpsSol.numSat < 5 ? 2 : 3;
      sats = gpsSol.numSat;
  }
  #endif
  buffer[0] = gpsFixType;
  buffer[1] = sats;
}
static void setGPSALT(uint8_t* buffer){
  int32_t alt = 0;
  #if defined(GPS)
    if (STATE(GPS_FIX) && sensors(SENSOR_GPS)) alt = gpsSol.llh.alt;
  #endif
  set32(buffer, alt);
}
static void setGPSDist(uint8_t* buffer){
  uint16_t dist = 0;
  #if defined(GPS)
    if (STATE(GPS_FIX) && sensors(SENSOR_GPS)) dist = GPS_distanceToHome;
  #endif
  set16u(buffer, dist);
}

static void setMode(uint8_t* buffer){
  uint16_t flightMode = 1; //Acro
  if (FLIGHT_MODE(ANGLE_MODE))  flightMode = 0; //Stab
  if (FLIGHT_MODE(BARO_MODE))   flightMode = 2; //AltHold
  if (FLIGHT_MODE(PASSTHRU_MODE)) flightMode = 3; //Auto
  if (FLIGHT_MODE(HEADFREE_MODE) || FLIGHT_MODE(MAG_MODE)) flightMode = 4; //Guided! (there in no HEAD, MAG so use Guided)
  if (FLIGHT_MODE(GPS_HOLD_MODE) && FLIGHT_MODE(BARO_MODE)) flightMode = 5; //Loiter
  if (FLIGHT_MODE(GPS_HOME_MODE)) flightMode = 6; //RTL
  if (FLIGHT_MODE(HORIZON_MODE)) flightMode = 7; //Circle! (there in no horizon so use Circle)
  if (FLIGHT_MODE(GPS_HOLD_MODE)) flightMode = 8; //PosHold
  if (FLIGHT_MODE(FAILSAFE_MODE)) flightMode = 9; //Land
  set16u(buffer, flightMode);
}
//2 bytes m/s *100
static void setTelemetryValueToBuffer(uint8_t* buffer, uint8_t sensorType, uint8_t length){
  	uint8_t i = 0;
  	uint8_t offset = 0;
    uint8_t size = 0;
    //clear buffer
  	for (i = 0; i < length; i++) {
  		buffer[i] = 0;
  	}
    switch (sensorType) {
      case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
        setVoltage(buffer);
        break;
      case IBUS_SENSOR_TYPE_TEMPERATURE:
        setTemperature(buffer);
        break;
      case IBUS_SENSOR_TYPE_RPM_FLYSKY:
        set16(buffer, (int16_t)rcCommand[THROTTLE]);
        break;
      case IBUS_SENSOR_TYPE_GPS_FULL:
        for (i = 0; i < sizeof(FULL_GPS_IDS); i++) {
          size = getSensorLength(FULL_GPS_IDS[i]);
          setTelemetryValueToBuffer(buffer + offset, FULL_GPS_IDS[i], size);
          offset += size;
        }
        break;
      case IBUS_SENSOR_TYPE_VOLT_FULL:
        for (i = 0; i < sizeof(FULL_VOLT_IDS); i++) {
            size = getSensorLength(FULL_VOLT_IDS[i]);
            setTelemetryValueToBuffer(buffer + offset, FULL_VOLT_IDS[i], size);
            offset += size;
        }
        break;
      case IBUS_SENSOR_TYPE_ACC_FULL:
        for (i = 0; i < sizeof(FULL_ACC_IDS); i++) {
          size = getSensorLength(FULL_ACC_IDS[i]);
          setTelemetryValueToBuffer(buffer + offset, FULL_ACC_IDS[i], size);
          offset += size;
        }
        break;
      case IBUS_SENSOR_TYPE_CELL:
        setCellVoltage(buffer);
        break;
      case IBUS_SENSOR_TYPE_BAT_CURR:
        setBatCurrent(buffer);
        break;
      case IBUS_SENSOR_TYPE_FUEL:
        setFuel(buffer);
        break;
      case IBUS_SENSOR_TYPE_RPM:
        setRPM(buffer);
        break;
      case IBUS_SENSOR_TYPE_CMP_HEAD:
        set16u(buffer, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;
      case IBUS_SENSOR_TYPE_VERTICAL_SPEED:
      case IBUS_SENSOR_TYPE_CLIMB_RATE:
      //same as climb rate!?!
        setVario(buffer);
        break;
      case IBUS_SENSOR_TYPE_COG:
        #if defined(GPS)
          if (STATE(GPS_FIX) && sensors(SENSOR_GPS))set16u(buffer, gpsSol.groundCourse * 100);
        #endif
        break;
      case IBUS_SENSOR_TYPE_GPS_STATUS:
        setGPSstatus(buffer);
        break;
      case IBUS_SENSOR_TYPE_ACC_X:
        set16(buffer, ((float)acc.accSmooth[X] / acc.dev.acc_1G) * 1000);
        //set16(buffer, accSum[X]);
        break;
      case IBUS_SENSOR_TYPE_ACC_Y:
        set16(buffer, ((float)acc.accSmooth[Y] / acc.dev.acc_1G) * 1000);
        break;
      case IBUS_SENSOR_TYPE_ACC_Z:
        set16(buffer, ((float)acc.accSmooth[Z] / acc.dev.acc_1G) * 1000);
        break;
      case IBUS_SENSOR_TYPE_ROLL:
        set16(buffer, attitude.values.roll * 10);
        break;
      case IBUS_SENSOR_TYPE_PITCH:
        set16(buffer, attitude.values.pitch * 10);
        break;
      case IBUS_SENSOR_TYPE_YAW:
        set16(buffer, attitude.values.yaw * 10);
        break;
      case IBUS_SENSOR_TYPE_GROUND_SPEED:
        #if defined(GPS)
          if (STATE(GPS_FIX) && sensors(SENSOR_GPS)) set16u(buffer, gpsSol.groundSpeed);
        #endif
        break;
      case IBUS_SENSOR_TYPE_ODO1:
      case IBUS_SENSOR_TYPE_ODO2:
      case IBUS_SENSOR_TYPE_GPS_DIST:
        setGPSDist(buffer);
        break;
      case IBUS_SENSOR_TYPE_ARMED:
        set16u(buffer, ARMING_FLAG(ARMED) ? 0 : 1);
        break;
      case IBUS_SENSOR_TYPE_FLIGHT_MODE:
        setMode(buffer);
        break;
      case IBUS_SENSOR_TYPE_PRES:
        set32u(buffer, baro.baroPressure |   ((uint32_t) (getTemperature() + IBUS_TEMPERATURE_OFFSET))  << 19);
        break;
      case IBUS_SENSOR_TYPE_SPE: //km/h
        //GPS_SPEED in cm/s => km/h, 1cm/s = 0.036 km/h
        #if defined(GPS)
          if (STATE(GPS_FIX) && sensors(SENSOR_GPS))   set16u(buffer, gpsSol.groundSpeed * 36 / 100);
        #endif
        break;
      case IBUS_SENSOR_TYPE_GPS_LAT:
        #if defined(GPS)
        if (STATE(GPS_FIX) && sensors(SENSOR_GPS))  set32u(buffer, gpsSol.llh.lat);
        #endif
        break;
      case IBUS_SENSOR_TYPE_GPS_LON:
        #if defined(GPS)
        if (STATE(GPS_FIX) && sensors(SENSOR_GPS))  set32u(buffer, gpsSol.llh.lon);
        #endif
        break;
      case IBUS_SENSOR_TYPE_GPS_ALT:
        setGPSALT(buffer);
        break;
      case IBUS_SENSOR_TYPE_ALT:
        set32(buffer, baro.BaroAlt);
        break;
      case IBUS_SENSOR_TYPE_ALT_MAX:
        set32(buffer, baro.BaroAlt);
        break;
    }
}
static void setIbusMeasurement(ibusAddress_t address)
{
    uint8_t sensorID = getSensorID(address);
    uint8_t sensorLength = getSensorLength(sensorID);
    sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE + sensorLength;
    sendBuffer[1] = IBUS_COMMAND_MEASUREMENT | address;
    setTelemetryValueToBuffer(sendBuffer + 2, sensorID, sensorLength);
}

static bool isCommand(ibusCommand_e expected, const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0xF0) == expected;
}

static ibusAddress_t getAddress(const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0x0F);
}

static void autodetectFirstReceivedAddressAsBaseAddress(ibusAddress_t returnAddress)
{
    if ((INVALID_IBUS_ADDRESS == ibusBaseAddress) &&
        (INVALID_IBUS_ADDRESS != returnAddress)) {
        ibusBaseAddress = returnAddress;
    }
}

static bool theAddressIsWithinOurRange(ibusAddress_t returnAddress)
{
    return (returnAddress >= ibusBaseAddress) &&
           (ibusAddress_t)(returnAddress - ibusBaseAddress) < ARRAYLEN(sensorAddressTypeLookup);
}

uint8_t respondToIbusRequest(uint8_t const * const ibusPacket)
{
    ibusAddress_t returnAddress = getAddress(ibusPacket);
    autodetectFirstReceivedAddressAsBaseAddress(returnAddress);
    //set out buffer to invalid
    sendBuffer[0] = INVALID_IBUS_ADDRESS;

    if (theAddressIsWithinOurRange(returnAddress)) {
        if (isCommand(IBUS_COMMAND_DISCOVER_SENSOR, ibusPacket)) {
            setIbusDiscoverSensorReply(returnAddress);
        } else if (isCommand(IBUS_COMMAND_SENSOR_TYPE, ibusPacket)) {
            setIbusSensorType(returnAddress);
        } else if (isCommand(IBUS_COMMAND_MEASUREMENT, ibusPacket)) {
            setIbusMeasurement(returnAddress);
        }
    }
    //trnsmit if content was set
    return transmitIbusPacket();
}


void initSharedIbusTelemetry(serialPort_t *port)
{
    ibusSerialPort = port;
    ibusBaseAddress = INVALID_IBUS_ADDRESS;
    memcpy(sensorAddressTypeLookup, telemetryConfig()->flysky_sensors, sizeof(telemetryConfig()->flysky_sensors));
}


#endif //defined(TELEMETRY) && defined(TELEMETRY_IBUS)

static uint16_t calculateChecksum(const uint8_t *ibusPacket)
{
    uint16_t checksum = 0xFFFF;
    uint8_t dataSize = ibusPacket[0] - IBUS_CHECKSUM_SIZE;
    for (size_t i = 0; i < dataSize; i++) {
        checksum -= ibusPacket[i];
    }

    return checksum;
}

bool isChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length)
{
    uint16_t calculatedChecksum = calculateChecksum(ibusPacket);

    // Note that there's a byte order swap to little endian here
    return (calculatedChecksum >> 8) == ibusPacket[length - 1]
           && (calculatedChecksum & 0xFF) == ibusPacket[length - 2];
}
