#include "fins_servo.h"
#include <PID_v1.h>
#include "../global.h"
#include "../configuration/configuration.h"

Servo servo_1; 
Servo servo_2; 
//Define Variables we'll be connecting to
double Setpoint_Pitch, Input_Pitch, Output_Pitch, Setpoint_Roll, Input_Roll, Output_Roll, Input_Yaw, Output_Yaw, Setpoint_Yaw;

PID pitchPID(&Input_Pitch, &Output_Pitch, &Setpoint_Pitch, _CONF.PID_PITCH_Kp, _CONF.PID_PITCH_Ki, _CONF.PID_PITCH_Kd, DIRECT);
PID yawPID(&Input_Yaw, &Output_Yaw, &Setpoint_Yaw, _CONF.PID_YAW_Kp, _CONF.PID_YAW_Ki, _CONF.PID_YAW_Kd, DIRECT);
PID rollPID(&Input_Roll, &Output_Roll, &Setpoint_Roll, _CONF.PID_ROLL_Kp, _CONF.PID_ROLL_Ki, _CONF.PID_ROLL_Kd, DIRECT);

void setupServo() {

    servo_1.attach(FINS_SERVO_1_PIN);  // attaches the servo pin
    servo_2.attach(FINS_SERVO_2_PIN);  // attaches the servo on pin 9 to the servo object
    
    servo_1.write(90 + _CONF.SERVO_1_OFFSET);                  
    servo_2.write(90 + _CONF.SERVO_2_OFFSET);  

    Setpoint_Pitch = 0;
    Setpoint_Roll = 0;

    //Specify the links and initial tuning parameters
    pitchPID.SetOutputLimits(_CONF.MAX_FINS_TRAVEL *-1, _CONF.MAX_FINS_TRAVEL);
    yawPID.SetOutputLimits(_CONF.MAX_FINS_TRAVEL *-1, _CONF.MAX_FINS_TRAVEL);
    //turn the PID on
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);
}

void testServo() {
    const int8_t test_amplitude = 30;

    // Test Servo #1
    // servo_1.write(90 + test_amplitude);   
    // servo_1.write(90 - test_amplitude);   
    // servo_1.write(90);   

    // // Test Servo #2
    // servo_2.write(90 + test_amplitude);   
    // servo_2.write(90 - test_amplitude);   
    // servo_2.write(85);   

    servo_1.write(0);
    servo_2.write(0);

}

// // OLD Direct Gyro-Servo connection
// void moveServo(float _ypr[]) {
// //ypr[0] * 180/M_PI

//     uint8_t pos_1;
//     uint8_t pos_2;

//     // pos_1 =(uint8_t) 90-((_ypr[1] * 180/M_PI)*-1);
//     pos_1 =(uint8_t) 90-((_ypr[1] * 180/M_PI) * SERVO_1_ORIENTATION);
//     pos_1 = ((pos_1 < 5) ? 5 : pos_1);
//     pos_1 = ((pos_1 >= 175) ? 175 : pos_1);

//     pos_2 =(uint8_t) 90-((_ypr[2] * 180/M_PI)  * SERVO_2_ORIENTATION); 
//     pos_2 = ((pos_2 < 5) ? 5 : pos_2);
//     pos_2 = ((pos_2 >= 175) ? 175 : pos_2);

//     Serial.print("servo-1: ");
//     Serial.print(pos_1);
//     Serial.print("   servo-2: ");
//     Serial.println(pos_2);
    
//     servo_1.write(pos_1);  
//     servo_2.write(pos_2);  
// }




void moveServo(float _ypr[]) {

    switch (_CONF.GUIDING_TYPE)
    {
        case GUIDING_TYPE_FINS:
            moveServoFins(_ypr);
            break;
        case GUIDING_TYPE_TVC:
            moveServoTVC(_ypr);
            break;
        default:
            // GUIDING_TYPE DISABLED
            break;
    }
}    


// NEW PID Controlled Gyro-Servo loop
void moveServoFins(float _ypr[]) {
    int16_t pos_1;
    int16_t pos_2;

    // Is this still necessary?
    // if(_ypr[_CONF.PITCH_AXIS] == 0 || _ypr[_CONF.YAW_AXIS] ==0) {
    //     // Data invalid do nothing
    //     return;
    // }

    pos_1 =(int16_t) (_ypr[_CONF.PITCH_AXIS] );
    pos_2 =(int16_t) (_ypr[_CONF.YAW_AXIS] );

    Input_Pitch = pos_1;
    Input_Yaw = pos_2;

    pitchPID.Compute();
    yawPID.Compute();

    g_servo_yaw = (Output_Yaw * _CONF.SERVO_2_ORIENTATION + 90) + _CONF.SERVO_2_OFFSET ;
    g_servo_pitch = (Output_Pitch * _CONF.SERVO_1_ORIENTATION + 90) + _CONF.SERVO_1_OFFSET ;

    if(_CONF.DEBUG) {
        Serial.print(millis());Serial.print(",");
        Serial.print(Input_Pitch);Serial.print(","); Serial.print(Input_Yaw);
        Serial.print(","); Serial.print(Output_Pitch * _CONF.SERVO_1_ORIENTATION);  // Reversed for graphing purposes
        Serial.print(","); Serial.print(Output_Yaw * _CONF.SERVO_2_ORIENTATION);    // Reversed for graphing purposes
        Serial.print(","); Serial.print(g_servo_pitch);
        Serial.print(","); Serial.print(g_servo_yaw);
        Serial.println("");
    }

    // Send the commands to the Servos
    servo_2.write(g_servo_yaw); 
    servo_1.write(g_servo_pitch);  
}

void moveServoTVC(float _ypr[]) {
    int16_t pos_1;
    int16_t pos_2;

    // Is this still necessary?
    // if(_ypr[_CONF.PITCH_AXIS] == 0 || _ypr[_CONF.YAW_AXIS] ==0) {
    //     // Data invalid do nothing
    //     return;
    // }

    pos_1 =(int16_t) (_ypr[_CONF.PITCH_AXIS] );
    pos_2 =(int16_t) (_ypr[_CONF.YAW_AXIS] );

    Input_Pitch = pos_1;
    Input_Yaw = pos_2;

    pitchPID.Compute();
    yawPID.Compute();

    g_servo_yaw = (Output_Yaw * _CONF.SERVO_2_ORIENTATION + 90) + _CONF.SERVO_2_OFFSET ;
    g_servo_pitch = (Output_Pitch * _CONF.SERVO_1_ORIENTATION + 90) + _CONF.SERVO_1_OFFSET ;

    if(_CONF.DEBUG) {
        Serial.print(millis());Serial.print(",");
        Serial.print(Input_Pitch);Serial.print(","); Serial.print(Input_Yaw);
        Serial.print(","); Serial.print(Output_Pitch * _CONF.SERVO_1_ORIENTATION);  // Reversed for graphing purposes
        Serial.print(","); Serial.print(Output_Yaw * _CONF.SERVO_2_ORIENTATION);    // Reversed for graphing purposes
        Serial.print(","); Serial.print(g_servo_pitch);
        Serial.print(","); Serial.print(g_servo_yaw);
        Serial.println("");
    }

    // Send the commands to the Servos
    servo_2.write(g_servo_yaw); 
    servo_1.write(g_servo_pitch);
    
}


// void adjustServo(servoNumber, adjustment) {

// }