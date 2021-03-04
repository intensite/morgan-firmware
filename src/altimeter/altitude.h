// #include "I2Cdev.h"
// #include "SparkFunMPL3115A2.h"
// #include "SparkFunBME280.h"
#include "../lib/SparkFunBME280.h"


class Altitude {
    private:
        BME280 myPressure;

    public:
        float altitude_offset=0;
        float pressure_offset=0;
        float current_altitude;
        float previous_altitude;
        bool is_apogee;
        float altitude_max;
        uint32_t start_descent_timer;
        float temperature;
        float pressure;
        float humidity;

        int16_t setupAlti();
        float processAltiData();
        Altitude();
        bool detectApogee();
        bool piroAltCheck(byte channel) ;
        bool detectChuteAltitude();
        bool detectTouchDown();
        float readFloatAltitudeMeters();
};