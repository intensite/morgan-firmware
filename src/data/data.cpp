#include "Arduino.h"
#include "../storage/Storage.h"
#include "../storage/LogSystem.h"
#include "../bluetooth/bluetooth.h"

void readDataToSerial() {
    uint32_t reccount = 0;
    reccount = lr::LogSystem::currentNumberOfRecords();
    Serial.print("Record Count : "); Serial.println(reccount);
        
    if (reccount > 0) {
        for(uint32_t i = 0; i < reccount; i++) {
            lr::LogRecord logRecord = lr::LogSystem::getLogRecord(i);
            logRecord.writeToSerial();
        }
        Serial.print("Ouff I just read xx records : "); Serial.println(reccount);
    } else {
        Serial.println("Nothing to read");
    }
}
//https://devzone.nordicsemi.com/index.php/dealing-large-data-packet-s-through-ble#reply-1755%22


void readDataToBLE() {
    uint32_t reccount = 0;
    reccount = lr::LogSystem::currentNumberOfRecords();
    Serial.print("Record Count : "); Serial.println(reccount);

    // Debug
    //reccount = 10;
        
    if (reccount > 0) {
        for(uint32_t i = 0; i < reccount; i++) {
            lr::LogRecord logRecord = lr::LogSystem::getLogRecord(i);
            // logRecord.writeToSerial();
            uploadFlightData(logRecord);
            delay(100);
            Serial.print("Sending Rec#: ");Serial.println(i);
        }
        Serial.print("Ouff I just read xx records to BLE : "); Serial.println(reccount);
    } else {
        Serial.println("Nothing to read");
    }

}
