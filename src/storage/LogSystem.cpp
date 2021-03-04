//
// Lucky Resistor's Deluxe Data Logger
// ---------------------------------------------------------------------------
// (c)2015 by Lucky Resistor. See LICENSE for details.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
#include "LogSystem.h"
#include "Storage.h"
#include "../configuration/configuration.h"
#include "../lib/PString.h"
#include "../lib/Streaming.h"

// #include <util/crc16.h>


namespace lr {


// LogRecord::LogRecord()
//     : _dateTime(), _temperature(0.0f), _humidity(0.0f)
// {    
// }

// Default null record constructor
LogRecord::LogRecord() 
        : _timestamp(0), _state(0), _altitude(0), _pitch(0), _roll(0), 
        _pitchServo(0), _rollServo(0), _parachute(0), _abort(0), 
        _temperature(0), _battery(0), _gForces(0)
{

}

// constructor
LogRecord::LogRecord(unsigned long timestamp, byte state, unsigned int altitude, int pitch, int roll, 
                int pitchServo, int rollServo, bool parachute, bool abort, 
                byte temperature, byte battery, byte gForces)
        : _timestamp(timestamp), _state(state), _altitude(altitude), _pitch(pitch), _roll(roll), 
        _pitchServo(pitchServo), _rollServo(rollServo), _parachute(parachute), _abort(abort), 
        _temperature(temperature), _battery(battery), _gForces(gForces)
{
    // Make data adjustments here if needed

}

// Destructor
LogRecord::~LogRecord()
{
}




bool LogRecord::isNull() const
{
    return _timestamp == 0 && _temperature == 0.0f;
}


void LogRecord::writeToSerial() const
{
    Serial.print(_timestamp);
    Serial.print(",");
    Serial.print(_state);
    Serial.print(",");
    Serial.print(_altitude);
    Serial.print(",");
    Serial.print(_pitch);
    Serial.print(",");
    Serial.print(_roll);
    Serial.print(",");
    Serial.print(_pitchServo);
    Serial.print(",");
    Serial.print(_rollServo);
    Serial.print(",");
    Serial.print(_parachute);
    Serial.print(",");
    Serial.print(_abort);
    Serial.print(",");
    Serial.print(_temperature);
    Serial.print(",");
    Serial.print(_battery);
    Serial.print(",");
    Serial.println(_gForces);
}
void LogRecord::writeToCSV(char** rec, size_t *rec_len) 
{
    /* This function knows the array will be of length RECORD_SIZE */
    // char buffer[100];
    char *buffer = (char*)malloc(100);

    PString str(buffer, sizeof(buffer));

    str << _timestamp << ",";
    str << _state << ",";
    str << _altitude << ",";
    str << _pitch << ",";
    str << _roll << ",";
    str << _pitchServo << ",";
    str << _rollServo << ",";
    str << _parachute << ",";
    str << _abort << ",";
    str << _temperature << ",";
    str << _battery << ",";
    str << _gForces << "\n";
 
    *rec = buffer;
    *rec_len = str.length();
    Serial.print(buffer);
}


namespace LogSystem {


static uint32_t gReservedForConfig; ///< The number of bytes reserved for the settings.
static uint32_t gCurrentNumberOfRecords; ///< The current number of records.
static uint32_t gMaximumNumberOfRecords; ///< The maximum number of records.


// The internal representation of a log record.
//
struct InternalLogRecord
{
    // uint32_t time; // The time as seconds since 2000-01-01 00:00:00.
    // float humidity; // The humidity value from the sensor.
    // float temperature; // The humidity value from the sensor.
    // uint16_t crc; // The CRC-16 of the record.

    unsigned long timestamp;    // Milliseconds since start
    byte state;    // Milliseconds since start
    unsigned int altitude;
    int pitch;
    int roll;
    int pitchServo;
    int rollServo;
    bool parachute;
    bool abort;
    byte temperature;
    byte battery;               // remaining volts x 10 
    byte gForces;
};

    
// Calculate the start of a record.
//
inline uint32_t getRecordStart(uint32_t index)
{
    return gReservedForConfig + (sizeof(InternalLogRecord) * index);
}

    
// Read one single internal record from the storage.
//
// @param index The index of the record.
// @return A copy of the internal record.
//
inline InternalLogRecord getInternalRecord(uint32_t index)
{
    InternalLogRecord record;
    Storage::readBytes(getRecordStart(index), reinterpret_cast<uint8_t*>(&record), sizeof(InternalLogRecord));
    return record;
}


// Write a single internal record to the storage.
//
// @param record The record to store.
// @param index The index of the record.
//
inline void setInternalRecord(const InternalLogRecord *record, uint32_t index)
{
    Storage::writeBytes(getRecordStart(index), reinterpret_cast<const uint8_t*>(record), sizeof(InternalLogRecord));
}
    

// Set a record in the storage to zero.
//
// @param index The index of the record to zero.
//
void zeroInternalRecord(uint32_t index)
{
    // Commented out as a not applicable to flash

    // uint32_t storageIndex = getRecordStart(index);
    // for (uint8_t i = 0; i < sizeof(InternalLogRecord); ++i) {
    //     Storage::writeByte(storageIndex, 0);
    //     ++storageIndex;
    // }
}
    

// Check if an internal record is null.
//
// This is true if all bytes of the records are null.
//
// @param record The record to check.
// @return true if the record is null.
//
bool isInternalRecordNull(InternalLogRecord *record)
{
    uint8_t *recordPtr = reinterpret_cast<uint8_t*>(record);
    for (uint8_t i = 0; i < sizeof(InternalLogRecord); ++i) {
        // if (*recordPtr != 0) {
        if (*recordPtr != 0xFF) {  // In the case of a flash chip the record is empty if all it bytes are set to 0xFF
            return false;
        }
        ++recordPtr;
    }
    return true;
}

    
// Calculate the CRC for the record.
//
// The CRC is calculated as CRC-16 while the CRC field is set to 0.
// All nibbles of the CRC-16 combined by XOR.
//
// @param record The record to calculate the CRC for.
// @return The 4-bit CRC
//
// uint16_t getCRCForInternalRecord(InternalLogRecord *record)
// {
//     // uint16_t crc = 0xFFFF;
//     // InternalLogRecord recordForCRC = *record;
//     // recordForCRC.crc = 0;
//     // uint8_t *recordPtr = reinterpret_cast<uint8_t*>(&recordForCRC);
//     // for (uint8_t i = 0; i < sizeof(InternalLogRecord); ++i) {
//     //     crc = _crc16_update(crc, *recordPtr);
//     //     ++recordPtr;
//     // }
//     // return crc;
// }
    
    
// Check if an internal record is valid.
//
// This is true if all values of the record are in a valid range
// and the CRC code is valid.
//
// @param record The record to check.
// @return true if the record is valid.
//
bool isInternalRecordValid(InternalLogRecord *record)
{
    // if (record->humidity < 0.0f ||
    //     record->humidity > 100.0f ||
    //     record->temperature < -273.15f ||
    //     record->temperature > 100.0f) {
    //     return false; // out of range.
    // }
    // const uint16_t crc = getCRCForInternalRecord(record);
    // return crc == record->crc;
    return true; // Debug only
}
    
    
void begin(uint32_t reservedForConfig)
{
    gReservedForConfig = reservedForConfig;
    gCurrentNumberOfRecords = 0;
    gMaximumNumberOfRecords = 0;
    
    // Calculate the maximum number of records.
    gMaximumNumberOfRecords = (Storage::size() - gReservedForConfig) / sizeof(InternalLogRecord);
    // Serial.println("DEBUG*******Get gMaximumNumberOfRecords *************************");
    // Serial.println(gMaximumNumberOfRecords);
    // Serial.print("sizeof(InternalLogRecord) : "); Serial.println(sizeof(InternalLogRecord));
    
    // Scan the storage for valid records.
    uint32_t index = 0;
    InternalLogRecord record = getInternalRecord(index);
    
    while (!isInternalRecordNull(&record) && index <= gMaximumNumberOfRecords) {
        if (!isInternalRecordValid(&record)) {
            break;
        }
        ++index;
        record = getInternalRecord(index);
    }
    if(index >= gMaximumNumberOfRecords) {
        Serial.println("############ MEMORY FULL #######################");
    }

    Serial.print("CurrentNumberOfRecords : "); Serial.println(index);
    
    gCurrentNumberOfRecords = index;
}


LogRecord getLogRecord(uint32_t index) {

    if (index >= gCurrentNumberOfRecords) {
        Serial.println("index is >= gCurrentNumberOfRecords : ");
        
        return LogRecord();
    }
    const InternalLogRecord record = getInternalRecord(index);
    return LogRecord(record.timestamp, record.state, record.altitude, record.pitch, 
                    record.roll, record.pitchServo, record.rollServo, 
                    record.parachute, record.abort, record.temperature, record.battery, record.gForces);
}


bool appendRecord(const LogRecord &logRecord)
{
    if (gCurrentNumberOfRecords >= gMaximumNumberOfRecords) {
        return false;
    }
    // zero the following record if possible
    // if (gCurrentNumberOfRecords+1 < gMaximumNumberOfRecords) {
    //     zeroInternalRecord(gCurrentNumberOfRecords+1);
    // }
    // convert the record into the internal structure.
    InternalLogRecord internalRecord;
    memset(&internalRecord, 0, sizeof(InternalLogRecord));
    internalRecord.timestamp = logRecord._timestamp;
    internalRecord.state = logRecord._state;
    internalRecord.altitude = logRecord._altitude;
    internalRecord.pitch = logRecord._pitch;
    internalRecord.roll = logRecord._roll;
    internalRecord.pitchServo = logRecord._pitchServo;
    internalRecord.rollServo = logRecord._rollServo;
    internalRecord.parachute = logRecord._parachute;
    internalRecord.abort = logRecord._abort;
    internalRecord.temperature = logRecord._temperature;
    internalRecord.battery = logRecord._battery;
    internalRecord.gForces = logRecord._gForces;

    // internalRecord.crc = getCRCForInternalRecord(&internalRecord);
    setInternalRecord(&internalRecord, gCurrentNumberOfRecords);
    gCurrentNumberOfRecords++;
    return true;
}

/*****************************************************************
 * Save a distinctive record to mark the start of a flight
 */
bool markBeginingOfDataSet() {

    LogRecord logRecord(99999,9,99999, 999, 999, 
                999, 999, false, false, 
               99, 99, 99);

    return appendRecord(logRecord);
}

void format()
{
    // zeroInternalRecord(0);
    // zeroInternalRecord(1);
    Storage::chipErase();
    gCurrentNumberOfRecords = 0;

}

bool isBusy() {
    return Storage::isBusy();
}
    
uint32_t maximumNumberOfRecords()
{
    return gMaximumNumberOfRecords;
}

    
uint32_t currentNumberOfRecords()
{
    Serial.println("Inside currentNumberOfRecords");
    Serial.print("Inside currentNumberOfRecords _CONF.DEBUG: "); Serial.println(_CONF.DEBUG);
    
    return gCurrentNumberOfRecords;
}


}
}  // Namespace crap 






