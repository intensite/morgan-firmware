#include <Arduino.h>
#include "parachute.h"
#include "../config.h"
#include "../led_color/led_color.h"
#include "../configuration/configuration.h"
// #include "../lib/TaskSchedulerDeclarations.h"
// #include "../global.h"

#define PYROTIME    3
// Scheduler
// Scheduler tsch;

// void PyroChannel_1_Off_cb();
// void PyroChannel_2_Off_cb();
// void PyroChannel_3_Off_cb();
// void PyroChannel_4_Off_cb();
// Task tPyroChannel_1_Off (PYROTIME*TASK_SECOND, TASK_ONCE, &ts, &PyroChannel_1_Off_cb);
// Task tPyroChannel_2_Off (PYROTIME*TASK_SECOND, TASK_ONCE, &ts, &PyroChannel_2_Off_cb);
// Task tPyroChannel_3_Off (PYROTIME*TASK_SECOND, TASK_ONCE, &ts, &PyroChannel_3_Off_cb);
// Task tPyroChannel_4_Off (PYROTIME*TASK_SECOND, TASK_ONCE, &ts, &PyroChannel_4_Off_cb);


bool deployParachute() {


    //TODO: Change parachute deployment logic to use one of the 4 customizable pyro channel 

    Serial.println("POP!!  Parachute deployed");
    
    // digitalWrite(PARACHUTE_IGNITER_PIN, true);


//TODO:  Improve this portion
    if (_CONF.PYRO_1_FIRE_ALTITUDE != 0) {
        activatePyro(1);
    }
    if (_CONF.PYRO_2_FIRE_ALTITUDE != 0) {
        activatePyro(2);
    }
    if (_CONF.PYRO_3_FIRE_ALTITUDE != 0) {
        activatePyro(3);
    }
    if (_CONF.PYRO_4_FIRE_ALTITUDE != 0) {
        activatePyro(4);
    }


    led_color(LED_COLOR_YELLOW);
    return true;
}

//TODO: Change parachute deployment logic to use one of the 4 customizable pyro channel 
void activatePyro(uint8_t channelNum) {
    //@TODO: Think of a way to hold the mosfet ON for a specific ammount of time

    Serial.print("Firing channel #");Serial.println(channelNum);
    switch(channelNum) {
        case 1:
            digitalWrite(PYRO_CHANEL_1, HIGH);
            //tPyroChannel_1_Off.enableDelayed();
            break;
        case 2:
            digitalWrite(PYRO_CHANEL_2, HIGH);
            //tPyroChannel_2_Off.enableDelayed();
            break;
        case 3:
            digitalWrite(PYRO_CHANEL_3, HIGH);
            //tPyroChannel_3_Off.enableDelayed();
            break;
        case 4:
            digitalWrite(PYRO_CHANEL_4, HIGH);
            //tPyroChannel_4_Off.enableDelayed();
            break;
    }
}

void resetPyro() {
    digitalWrite(PYRO_CHANEL_1, LOW);
    digitalWrite(PYRO_CHANEL_2, LOW);
    digitalWrite(PYRO_CHANEL_3, LOW);
    digitalWrite(PYRO_CHANEL_4, LOW);
}

//@TODO: Optimize these call back function into just one
void PyroChannel_1_Off_cb() {
    digitalWrite(PYRO_CHANEL_1, LOW);
    Serial.println("Channel #1 powered OFF");
}
void PyroChannel_2_Off_cb(){
    digitalWrite(PYRO_CHANEL_2, LOW);
    Serial.println("Channel #2 powered OFF");
}
void PyroChannel_3_Off_cb(){
    digitalWrite(PYRO_CHANEL_3, LOW);
    Serial.println("Channel #3 powered OFF");
}
void PyroChannel_4_Off_cb(){
    digitalWrite(PYRO_CHANEL_4, LOW);
    Serial.println("Channel #4 powered OFF");
}