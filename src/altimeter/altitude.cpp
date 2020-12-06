
/* ====================================================================
 * 2019 Stephen Remillard / Francois Paquette
 * based on the excelent I2Cdev device library code from Jeff Rowberg
 * available at https://github.com/jrowberg/i2cdevlib
 ====================================================================== */


#include "../config.h"
#include "altitude.h"
#include "../lib/I2Cdev.h"
#include "../parachute/parachute.h"
#include "../global.h"
#include "../lib/SimpleKalmanFilter.h"
#include "../configuration/configuration.h"




// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// #define SDA 23 //0
// #define SCL 22 //4 

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.3);

Altitude::Altitude() {};

int16_t Altitude::setupAlti() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        // Wire.begin();
        // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.begin(SDA, SCL, 400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        // Fastwire::setup(400, true);
    #endif

    myPressure.setI2CAddress(0x76); //The default for the SparkFun Environmental Combo board is 0x77 (jumper open).
    altitude_max = 0; previous_altitude = 0; is_apogee = false;

    //Configure the sensor
    myPressure.setFilter(0);
    myPressure.setMode(3); //Normal mode
    myPressure.setStandbyTime(0);
    myPressure.setPressureOverSample(1);
    myPressure.setTempOverSample(1);
    // myPressure.setReferencePressure(100000); // Local Atmospheric Pressure  ex: 99,9Kpa
    myPressure.setReferencePressure(99700); // Local Atmospheric Pressure  ex: 99,9Kpa

    delay(1000);
    // initialize device
    myPressure.beginI2C(); // Get sensor online

    temperature = myPressure.readTempC();
    Serial.println(F("Stabilisation current altitude...."));
    // The BMx280 'saves' the last reading in memory for you to query. Just read twice in a row and toss out the first reading!
    for(int8_t i=0; i<20; i++) {
        temperature = myPressure.readTempC();
        altitude_offset = myPressure.readFloatAltitudeMeters(); 
        Serial.print(F("Temperature=")); Serial.print(myPressure.readTempC(), 2);
        Serial.print(F("  Altitude Offset=")); Serial.println(altitude_offset);
        delay(500);
    }
    Serial.print(F("Altitude Offset = ")); Serial.println(altitude_offset); 
    if (altitude_offset == -999) {
        return -999;  // Error out after max of 512ms for a read
    } else {
        return 0;
    }
}



float Altitude::processAltiData() {

    // Get the current altitude using the altitude_offset
    current_altitude = myPressure.readFloatAltitudeMeters() - altitude_offset;
    current_altitude = pressureKalmanFilter.updateEstimate(current_altitude);
    // Serial.print(F("current_altitude = ")); Serial.println(current_altitude); 

    temperature = myPressure.readTempC();
    pressure = myPressure.readFloatPressure() / 1000;
    humidity = myPressure.readFloatHumidity();
    
    // Ignore negative altitude
    if (current_altitude < 0) {
        current_altitude = 0;
    }

    // Record the max altitude (overide)
    if (current_altitude > altitude_max){
        altitude_max = current_altitude;
    }

    // Replace the previous_altitude with the current altitude for next pass.
    previous_altitude = current_altitude;
    return current_altitude;
}

bool Altitude::detectApogee() {

    // Separate the appogee detection from the reading of the sensor for more flexibility
    // Check if Apogee was reached and beginig descent
    if(altitude_max > _CONF.APOGEE_DIFF_METERS) {  // Prevent on the ground and transport accident

        if((current_altitude < altitude_max) && ((altitude_max - current_altitude) >= _CONF.APOGEE_DIFF_METERS))  {  
            // Here we should be going down.
            Serial.print(F("Apogee passed. Max altitude: "));
            Serial.println(altitude_max);
            // is_parachute_deployed = deployParachute();  //@TODO: This call should probably be moved out of this class
            return true;
        }
    }

    return false;
}


bool Altitude::detectChuteAltitude() {
    byte result = 0;

    if(altitude_max > _CONF.APOGEE_DIFF_METERS) {   // Prevent on the ground and transport accident
        if(detectApogee()) {                        // Make sure we have reached apogee
            // Check each pyro
            for(uint8_t i=1; i<4; i++){
                if(piroAltCheck(i)){                // Check each Pyro conditions
                    activatePyro(i);                // If it matches Fire the corresponding Pyro Chanel
                    result++;
                }
            }
        }
    }
    return (result > 0);                            // Return true if one of the pyro was fired
}

bool Altitude::piroAltCheck(byte channel) {
    //@TODO Find a better way to check for all PIRO fire altitudes

    // PYRO_XX_FIRE_ALTITUDE: 0= Innactive, -1= Apogee, xx=Altitude in metters
    switch (channel ) {
        case 1:
            if (_CONF.PYRO_1_FIRE_ALTITUDE != 0) {
                if(_CONF.PYRO_1_FIRE_ALTITUDE == -1)
                    // return detectApogee();
                    return true;
                else
                    return (current_altitude <=  _CONF.PYRO_1_FIRE_ALTITUDE);
            }
            break;
        case 2:
            if (_CONF.PYRO_2_FIRE_ALTITUDE != 0) {
                if(_CONF.PYRO_2_FIRE_ALTITUDE == -1)
                    // return detectApogee();
                    return true;
                else
                    return (current_altitude <=  _CONF.PYRO_2_FIRE_ALTITUDE);
            }
            break;
        case 3:
            if (_CONF.PYRO_3_FIRE_ALTITUDE != 0) {
                if(_CONF.PYRO_3_FIRE_ALTITUDE == -1)
                    // return detectApogee();
                    return true;
                else
                    return (current_altitude <=  _CONF.PYRO_3_FIRE_ALTITUDE);
            }
            break;
        case 4:
            if (_CONF.PYRO_4_FIRE_ALTITUDE != 0) {
                if(_CONF.PYRO_4_FIRE_ALTITUDE == -1)
                    // return detectApogee();
                    return true;
                else
                    return (current_altitude <=  _CONF.PYRO_4_FIRE_ALTITUDE);
            }
            break;
    }
    return false;
}

bool Altitude::detectTouchDown() {

    if(current_altitude <= 5)
        return true;
    else 
        return false;
}