#include "Arduino.h"
#include "simulation.h"
#include "../config.h"
#include "../configuration/configuration.h"
#include "../altimeter/altitude.h"
// #include "../gyro/gyro.h"



Simulation::Simulation() { }

void Simulation::handleReceivedMessage(char* msg) {

    Serial.println(msg);

    if(strcmp(msg, "ZZZZ") == 0) {
        _CONF.SIMULATION_MODE = 0;
        Serial.print("_CONF.SIMULATION_MODE: "); Serial.println(_CONF.SIMULATION_MODE);    
    } else {
        char * token = strtok(msg, ";");
        double accel = atof(token);
        double alti;
        if( token != NULL ) {
            // printf( " %s\n", token ); //printing each token
            token = strtok(NULL, ";");
            alti = atof(token);
        }

        //Gyro::z_gforce = accel;
        Altitude::current_altitude = alti;

    }

    
}