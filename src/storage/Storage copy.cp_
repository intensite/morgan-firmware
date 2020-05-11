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
//#include "I2Cdev.h"

// // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// // is used in I2Cdev.h
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
// #endif

namespace lr {
namespace Storage {

/// The address of the FRAM chip in the I2C bus.
///
/// The address is 1010AAA where AAA is the custom address which can
/// be set with the pins on the chip.
///
static const uint8_t cMb85RcAddress = B1010000;
// static const uint8_t cMb85RcAddress = (0x50);


bool begin()
{
    uint8_t result;

    // // join I2C bus (I2Cdev library doesn't do this automatically)
    // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //     Fastwire::setup(400, true);
    // #endif

    // Read the manufacturer ID and product ID to make sure the FRAM is available.
    Wire.beginTransmission(0xF8 >> 1);
	Wire.write((byte)(cMb85RcAddress << 1));
	result = Wire.endTransmission(false);

    Wire.requestFrom(0xf8>>1, 3);

    const uint8_t id0 = (uint8_t) Wire.read();
    const uint8_t id1 = (uint8_t) Wire.read();
    const uint8_t id2 = (uint8_t) Wire.read();
    const uint16_t manufacturerID = (id0<<4)+(id1>>4);
    const uint16_t productID = ((id1&0x0f)<<8)+id2;
    
    Serial.print(manufacturerID, HEX);
    Serial.print(", ");
    Serial.println(productID, HEX);
    // Check the both IDs
    if (manufacturerID != 0x00a || productID != 0x510) {
        return false;
    }

    Serial.println("FRAM seems Good!");
    return true;
}


uint32_t size()
{
    return 32768; // 32KB
}


void writeByte(uint32_t index, uint8_t data)
{
    Wire.beginTransmission(cMb85RcAddress);
    Wire.write(index>>8);
    Wire.write(index&0xff);
    Wire.write(data);
    Wire.endTransmission();
}


void writeBytes(uint32_t firstIndex, const uint8_t *data, uint32_t size)
{
    Wire.beginTransmission(cMb85RcAddress);
    Wire.write(static_cast<uint8_t>(firstIndex>>8));
    Wire.write(static_cast<uint8_t>(firstIndex&0xff));
    for (uint32_t i = 0; i < size; ++i) {
        Wire.write(data[i]);
    }
    Wire.endTransmission();
}


uint8_t readByte(uint32_t index)
{
    Wire.beginTransmission(cMb85RcAddress);
    Wire.write(static_cast<uint8_t>(index>>8));
    Wire.write(static_cast<uint8_t>(index&0xff));
    Wire.endTransmission();
    Wire.requestFrom(cMb85RcAddress, static_cast<uint8_t>(1));
    return Wire.read();
}


void readBytes(uint32_t firstIndex, uint8_t *data, uint32_t size)
{
    // reads >0xff will not work.
    Wire.beginTransmission(cMb85RcAddress);
    Wire.write(static_cast<uint8_t>(firstIndex>>8));
    Wire.write(static_cast<uint8_t>(firstIndex&0xff));
    Wire.endTransmission();
    Wire.requestFrom(cMb85RcAddress, static_cast<uint8_t>(size));
    for (uint32_t i = 0; i < size; ++i) {
        data[i] = Wire.read();
    }
}


}
}  // Name space crap 



