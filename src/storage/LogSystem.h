#pragma once
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


// #include "DateTime.h"

#include <Arduino.h>


namespace lr {


/// A single log record.
///
class LogRecord
{
public:
    /// Create a new log record using the given values.
    ///
    /// @param dateTime The time of the record.
    /// @param temperature The temperature in celsius.
    /// @param humidity The humidity as percentage 0-100.
    ///
    // LogRecord(const DateTime &dateTime, float temperature, float humidity);
    LogRecord(unsigned long timestamp, byte state, unsigned int altitude, int pitch, int roll, 
                int pitchServo, int rollServo, bool parachute, bool abort, 
                byte temperature, byte battery, byte gForces);
                
    /// Create a special null record.
    ///
    /// This records are used in error situations.
    ///
    LogRecord();
    
    /// dtor
    ///
    ~LogRecord();

public:
    /// Check if this is a null record.
    ///
    bool isNull() const;
    
    /// Get the time of the record.
    ///
    // inline DateTime getDateTime() const { return _dateTime; }
    
    /// Get the temperature of the record in celsius.
    ///
    // inline float getTemperature() const { return _temperature; }
    
    /// Get the humidity of the record in percent 0-100.
    ///
    // inline float getHumidity() const { return _humidity; }
    
    /// Write this record to the serial interface.
    ///
    /// The format is: date/time, temperature, humidity
    /// Example: 2015-08-22 12:42:21,80,25
    ///
    void writeToSerial() const;
    // void writeToCSV() const;
    void writeToCSV(char **rec, size_t *rec_len);
    
public:     // Public for now.  Maybe getter and setters would be more appropriate
    unsigned long _timestamp;  // Milliseconds since start
    byte _state;
    unsigned int _altitude;
    int _pitch;
    int _roll;
    int _pitchServo;
    int _rollServo;
    bool _parachute;
    bool _abort;
    byte _temperature;
    byte _battery; // remaining volts x 10 
    byte _gForces;
};



/// The log system to write and read all sensor data.
///
namespace LogSystem {


/// Initialize the log system
///
void begin(uint32_t reservedForConfig);

/// Get the maximum number of records for the given storage.
///
uint32_t maximumNumberOfRecords();

/// Get the number of records currently in the storage.
///
uint32_t currentNumberOfRecords();

/// Read a record from the storage.
///
LogRecord getLogRecord(uint32_t index);

/// Append a record to the storage.
///
/// This will first zero the record (index+1) if possible, before
/// writing the given record to (index).
///
/// @param logRecord The record to append.
/// @return true on success, false if the storage is full.
///
bool appendRecord(const LogRecord &logRecord);


/// Insert a special (9999) record to the storage.
///
/// This function will insert a single record with only 9s to disinguish a separate flight.
/// It uses appendRecord internally
///
bool markBeginingOfDataSet();

/// Format the storage.
///
/// This will set the initial two records of the storage area to zero.
/// It is enough to initialize the storage with minimum number of writes.
///
void format();

bool isBusy();
    

}
}   // Namespace crap 





