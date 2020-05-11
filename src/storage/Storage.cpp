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

#include "Storage.h"
#include "../lib/SPIFlash.h"    //get it here: https://github.com/LowPowerLab/SPIFlash

#define FLASH_SS      15 

namespace lr {
namespace Storage {

//////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF40 for windbond 64M-bit / 8M-byte flash
//////////////////////////////////////////
SPIFlash flash(FLASH_SS, 0xEF40);


bool begin() {
    if (flash.initialize()){
        Serial.println("Init OK!");
        return true;
    } else {
        Serial.println("Init FAIL!");
        return false;
    }
    
    // delay(1000);
}


uint32_t size() {
    return 8388608; // 8MB (8x1024x1024)
}


void writeByte(uint32_t index, uint8_t data){
     flash.writeByte(index, data);
}


void writeBytes(uint32_t firstIndex, const uint8_t *data, uint32_t size){
    // Serial.print("Writing n-bytes : "); Serial.println(size);
    
    flash.writeBytes(firstIndex, data, size);
}


uint8_t readByte(uint32_t index) {
    return flash.readByte(index);
}


void readBytes(uint32_t firstIndex, uint8_t *data, uint32_t size) {
    // reads >0xff will not work.
    flash.readBytes(firstIndex, data, size);
}

/// erase entire flash memory array
/// may take several seconds depending on size, but is non blocking
void chipErase() {
    Serial.println("Erasing memory.........");
    flash.chipErase();
}

bool isBusy() {
    return flash.busy();
}

}
}  // Name space crap 



