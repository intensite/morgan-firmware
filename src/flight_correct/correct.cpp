
#include "correct.h"
#include "../global.h"
#include "../parachute/parachute.h"
#include "../configuration/configuration.h"

void processTrajectory(float ypr[3]) {
    int8_t pos_1;
    int8_t pos_2;

    // Serial.println("InsideprocessTrajectory");
    // Serial.print("ypr[_CONF.PITCH_AXIS]: "); Serial.println(ypr[_CONF.PITCH_AXIS]);
    // Serial.print("ypr[_CONF.YAW_AXIS]: "); Serial.println(ypr[_CONF.YAW_AXIS]);
    // Serial.print("ypr[_CONF.ROLL_AXIS]: "); Serial.println(ypr[_CONF.ROLL_AXIS]);
    

    // if(ypr[_CONF.PITCH_AXIS] == 0 || ypr[_CONF.YAW_AXIS] ==0) {
    //     // Data invalid do nothing
    //     return;
    // }

    // pos_1 =(int8_t) 90-(ypr[_CONF.PITCH_AXIS] * 180/M_PI);
    // pos_2 =(int8_t) 90-(ypr[_CONF.YAW_AXIS] * 180/M_PI);
    
    // pos_1 =(int8_t) 90-(ypr[_CONF.PITCH_AXIS] );
    // pos_2 =(int8_t) 90-(ypr[_CONF.YAW_AXIS] );


    if(_CONF.AUTOMATIC_ANGLE_ABORT) {
        //TODO: Compare angle to EXCESSIVE_ANGLE_THRESHOLD config constant to abort 
        if(pos_1 < (90 - _CONF.EXCESSIVE_ANGLE_THRESHOLD) || pos_1 > (90 + _CONF.EXCESSIVE_ANGLE_THRESHOLD) || 
            pos_2 < (90 - _CONF.EXCESSIVE_ANGLE_THRESHOLD) || pos_2 > (90 + _CONF.EXCESSIVE_ANGLE_THRESHOLD)) {
            // Here we initiate the abort sequence
            Serial.println("Excessive angle. ABORT SEQUENCE...........");
            Serial.print("pos_1: ");
            Serial.print(pos_1);
            Serial.print("  pos_2: ");
            Serial.print(pos_2);
            Serial.print("  ypr[_CONF.PITCH_AXIS]: ");
            Serial.print(ypr[_CONF.PITCH_AXIS]);
            Serial.print("  ypr[_CONF.YAW_AXIS]: ");
            Serial.println(ypr[_CONF.YAW_AXIS]);
            deployParachute();
            //is_abort = true;
            return;
        }
    }

    // Serial.println("Calling moveServo");
    moveServo(ypr);
}

