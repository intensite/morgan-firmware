#include "Arduino.h"
#include "command.h"
#include "../config.h"
#include "../buzzer/buzzer.h"
#include "../parachute/parachute.h"
#include "../configuration/configuration.h"
#include <SimpleCLI.h>


CliCommand::CliCommand() {
    cmdGet = cli.addCommand("get");
    cmdGet.addPositionalArgument("setting");
    // cmdGet.addPositionalArgument("value");

    cmdSet = cli.addCommand("set");
    cmdSet.addPositionalArgument("setting");
    cmdSet.addPositionalArgument("value");
}

void CliCommand::handleSerial() {
   const int BUFF_SIZE = 32; // make it big enough to hold your longest command
   static char buffer[BUFF_SIZE+1]; // +1 allows space for the null terminator
   static int length = 0; // number of characters currently in the buffer
//    static boolean newData = false;

    //    if(Serial.available())
    // while (Serial.available() > 0 && newData == false) {
    while (Serial.available() > 0 ) {
        char c = Serial.read();
        if((c == '\r') || (c == '\n')){
        // if(c == ';') {
            // end-of-line received
            Serial.println("end-of-line received");
            
            if(length > 0) {
                this->handleReceivedMessage(buffer);
            }
            length = 0;
            // newData = true;
        } else {
            if(length < BUFF_SIZE) {
                buffer[length++] = c; // append the received character to the array
                buffer[length] = 0; // append the null terminator
                // Serial.print("Caractere ajouté : "); Serial.println(c);
            } else {
                // buffer full - discard the received character
                Serial.println("buffer full");
                
            }
        }
    }
    // Serial.println("End of handleSerial() function...");
}

void CliCommand::handleReceivedMessage(char* msg)
{
    std::string str(msg);
    
    // Serial.println("Message received!");
    // Serial.println(msg);

    // Get rid on the double quotes from the msg string
    str.erase(std::remove(str.begin(),str.end(),'\"'),str.end());

    this->cli.parse(str.c_str());

    // Check if a new error occurred
    if(this->cli.errored()) {
    CommandError e = this->cli.getError();

    // Print the error, or do whatever you want with it
    Serial.println(e.toString());
    }

    // First check if a newly parsed command is available
    if(this->cli.available()) {
        // Serial.print("It appears to be a valid command!");

        // Get the command out of the queue
        Command cmd = this->cli.getCommand();

        // Get the Argument(s) you want
        Argument argSetting = cmd.getArgument("setting"); // via name
        Argument argValue = cmd.getArgument("value"); // via index
        // strcpy(setting, argSetting.getValue());
        // strcpy(value, argValue.getValue());
        String setting = argSetting.getValue();
        String value = argValue.getValue();


        // Check if it's the command you're looking for
        if(cmd == this->cmdGet) {
            this->processGetCommand(setting.c_str());
        } 
        if(cmd == this->cmdSet) {
            this->processSetCommand(setting.c_str(), value.c_str());
        }
    } else {

        Serial.print("It appears to be a INVALID command!");
    }

    return;
}

void CliCommand::processGetCommand(const char* setting) {
    if(strcmp(setting, "BUZZER") == 0) {
        Serial.print("_CONF.BUZZER_ENABLE: "); Serial.println(_CONF.BUZZER_ENABLE);    
    } 
    else if(strcmp(setting, "DEBUG") == 0) {
        Serial.print("_CONF.DEBUG: "); Serial.println(_CONF.DEBUG);    
    } 
    // else if(strcmp(setting, "buzzer") == 0) {
    // } 
    // else if(strcmp(setting, "buzzer") == 0) {
    // } 
    // else if(strcmp(setting, "buzzer") == 0) {
    // } 
    // else if(strcmp(setting, "buzzer") == 0) {
    // } 
    // else if(strcmp(setting, "buzzer") == 0) {
    // } 
    else if(strcmp(setting, "ALL") == 0) {
        Serial.print("_CONF.DEBUG: "); Serial.println(_CONF.DEBUG);    
        Serial.print("_CONF.BUZZER_ENABLE: "); Serial.println(_CONF.BUZZER_ENABLE);    
        Serial.print("_CONF.MEMORY_CARD_ENABLED: "); Serial.println(_CONF.MEMORY_CARD_ENABLED);    
        Serial.print("_CONF.DATA_RECOVERY_MODE: "); Serial.println(_CONF.DATA_RECOVERY_MODE);    
        Serial.print("_CONF.FORMAT_MEMORY: "); Serial.println(_CONF.FORMAT_MEMORY);    
        Serial.print("_CONF.SCAN_TIME_INTERVAL: "); Serial.println(_CONF.SCAN_TIME_INTERVAL);    

        // PYRO CONTROL
        Serial.println("// PYRO CONTROL");
        Serial.print("_CONF.APOGEE_DIFF_METERS: "); Serial.println(_CONF.APOGEE_DIFF_METERS);    
        Serial.print("_CONF.PARACHUTE_DELAY: "); Serial.println(_CONF.PARACHUTE_DELAY);    
        Serial.print("_CONF.PYRO_ACTIVATION_DELAY: "); Serial.println(_CONF.PYRO_ACTIVATION_DELAY);    
        Serial.print("_CONF.PYRO_1_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_1_FIRE_ALTITUDE);    
        Serial.print("_CONF.PYRO_2_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_2_FIRE_ALTITUDE);    
        Serial.print("_CONF.PYRO_3_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_3_FIRE_ALTITUDE);    
        Serial.print("_CONF.PYRO_4_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_4_FIRE_ALTITUDE);    
        Serial.print("_CONF.AUTOMATIC_ANGLE_ABORT: "); Serial.println(_CONF.AUTOMATIC_ANGLE_ABORT);    
        Serial.print("_CONF.EXCESSIVE_ANGLE_THRESHOLD: "); Serial.println(_CONF.EXCESSIVE_ANGLE_THRESHOLD);    
        Serial.print("_CONF.EXCESSIVE_ANGLE_TIME: "); Serial.println(_CONF.EXCESSIVE_ANGLE_TIME);    


        // GUIDANCE CONTROL
        Serial.println("// GUIDANCE CONTROL");
        Serial.print("_CONF.GUIDING_TYPE: "); Serial.println(_CONF.GUIDING_TYPE);    
        Serial.print("_CONF.ROLL_CONTROL_ENABLED: "); Serial.println(_CONF.ROLL_CONTROL_ENABLED);    
        Serial.print("_CONF.ROLL_CONTROL_TYPE: "); Serial.println(_CONF.ROLL_CONTROL_TYPE);    
        
        // SERVO AXIS MAPPING  (POSSIBLE VALUES: X,Y,Z,A)   // A Stands for Other Possibly for reaction wheel
        Serial.print("_CONF.SERVO_1_AXIS: "); Serial.println(_CONF.SERVO_1_AXIS);    
        Serial.print("_CONF.SERVO_2_AXIS: "); Serial.println(_CONF.SERVO_2_AXIS);    
        Serial.print("_CONF.SERVO_3_AXIS: "); Serial.println(_CONF.SERVO_3_AXIS);    
        Serial.print("_CONF.SERVO_4_AXIS: "); Serial.println(_CONF.SERVO_4_AXIS);    

        Serial.print("_CONF.SERVO_1_OFFSET: "); Serial.println(_CONF.SERVO_1_OFFSET);    
        Serial.print("_CONF.SERVO_2_OFFSET: "); Serial.println(_CONF.SERVO_2_OFFSET);    
        Serial.print("_CONF.SERVO_3_OFFSET: "); Serial.println(_CONF.SERVO_3_OFFSET);    
        Serial.print("_CONF.SERVO_4_OFFSET: "); Serial.println(_CONF.SERVO_4_OFFSET);    

        Serial.print("_CONF.SERVO_1_ORIENTATION: "); Serial.println(_CONF.SERVO_1_ORIENTATION);    
        Serial.print("_CONF.SERVO_2_ORIENTATION: "); Serial.println(_CONF.SERVO_2_ORIENTATION);    
        Serial.print("_CONF.SERVO_3_ORIENTATION: "); Serial.println(_CONF.SERVO_3_ORIENTATION);    
        Serial.print("_CONF.SERVO_4_ORIENTATION: "); Serial.println(_CONF.SERVO_4_ORIENTATION);    
        Serial.print("_CONF.MAX_FINS_TRAVEL: "); Serial.println(_CONF.MAX_FINS_TRAVEL);    

        // PID TUNING
        Serial.print("_CONF.PID_PITCH_Kp: "); Serial.println(_CONF.PID_PITCH_Kp);    
        Serial.print("_CONF.PID_PITCH_Ki: "); Serial.println(_CONF.PID_PITCH_Ki);    
        Serial.print("_CONF.PID_PITCH_Kd: "); Serial.println(_CONF.PID_PITCH_Kd);    
        Serial.print("_CONF.PID_YAW_Kp: "); Serial.println(_CONF.PID_YAW_Kp);    
        Serial.print("_CONF.PID_YAW_Ki: "); Serial.println(_CONF.PID_YAW_Ki);    
        Serial.print("_CONF.PID_YAW_Kd: "); Serial.println(_CONF.PID_YAW_Kd);    
        Serial.print("_CONF.PID_ROLL_Kp: "); Serial.println(_CONF.PID_ROLL_Kp);    
        Serial.print("_CONF.PID_ROLL_Ki: "); Serial.println(_CONF.PID_ROLL_Ki);    
        Serial.print("_CONF.PID_ROLL_Kd: "); Serial.println(_CONF.PID_ROLL_Kd);   

        // IMU AXIS PHYSICAL LOCATION (ypr[3])
        Serial.print("_CONF.PITCH_AXIS: "); Serial.println(_CONF.PITCH_AXIS);    
        Serial.print("_CONF.YAW_AXIS: "); Serial.println(_CONF.YAW_AXIS);    
        Serial.print("_CONF.ROLL_AXIS: "); Serial.println(_CONF.ROLL_AXIS);    

        // IMU CALIBRATION
        Serial.print("_CONF.X_GYRO_OFFSETS: "); Serial.println(_CONF.X_GYRO_OFFSETS);    
        Serial.print("_CONF.Y_GYRO_OFFSETS: "); Serial.println(_CONF.Y_GYRO_OFFSETS);    
        Serial.print("_CONF.Z_GYRO_OFFSETS: "); Serial.println(_CONF.Z_GYRO_OFFSETS);    
        Serial.print("_CONF.X_ACCEL_OFFSETS: "); Serial.println(_CONF.X_ACCEL_OFFSETS);    
        Serial.print("_CONF.Y_ACCEL_OFFSETS: "); Serial.println(_CONF.Y_ACCEL_OFFSETS);    
        Serial.print("_CONF.Z_ACCEL_OFFSETS: "); Serial.println(_CONF.Z_ACCEL_OFFSETS);    
        Serial.print("_CONF.VERSION: "); Serial.println(_CONF.VERSION);    
    }
}

void CliCommand::processSetCommand(const char* setting, const char* value) {
    uint8_t DO_NOT_SAVE_FLAG = false;       // To save  the SPIFFS memory freom unnecessary write access
    
    // ----------------- PREFS PAGE -----------------------------------
    if(strcmp(setting, "BUZZER") == 0) {
        _CONF.BUZZER_ENABLE = atoi(value); 
        Serial.print("_CONF.BUZZER_ENABLE: "); Serial.println(_CONF.BUZZER_ENABLE);  
    } 
    else if(strcmp(setting, "DEBUG") == 0) {
        _CONF.DEBUG = atoi(value); 
        Serial.print("_CONF.DEBUG: "); Serial.println(_CONF.DEBUG);    
    } 
    else if(strcmp(setting, "MEM_ENABLED") == 0) {
        _CONF.MEMORY_CARD_ENABLED = atoi(value); 
        Serial.print("_CONF.MEMORY_CARD_ENABLED: "); Serial.println(_CONF.MEMORY_CARD_ENABLED);    
    } 
    else if(strcmp(setting, "DATA_MODE") == 0) {
        _CONF.DATA_RECOVERY_MODE = atoi(value); 
        Serial.print("_CONF.DATA_RECOVERY_MODE: "); Serial.println(_CONF.DATA_RECOVERY_MODE);    
    } 
    else if(strcmp(setting, "FORMAT_MEM") == 0) {
        _CONF.FORMAT_MEMORY = atoi(value); 
        Serial.print("_CONF.FORMAT_MEMORY: "); Serial.println(_CONF.FORMAT_MEMORY);    
    } 
    else if(strcmp(setting, "SCAN_TIME_INTERVAL") == 0) {
        _CONF.SCAN_TIME_INTERVAL = atoi(value); 
        Serial.print("_CONF.SCAN_TIME_INTERVAL: "); Serial.println(_CONF.SCAN_TIME_INTERVAL);    
    } 
    // ----------------- DIAGS PAGE -----------------------------------
    else if(strcmp(setting, "ARMED_STATUS") == 0) {
        DO_NOT_SAVE_FLAG = true;
        _CONF.ARMED_STATUS = atoi(value); 
        Serial.print("_CONF.ARMED_STATUS: "); Serial.println(_CONF.ARMED_STATUS);    
    } 

    // ----------------- PYRO PAGE -----------------------------------
    else if(strcmp(setting, "PYRO_ACTIVATION_DELAY") == 0) {
        _CONF.PYRO_ACTIVATION_DELAY = atoi(value); 
        Serial.print("_CONF.PYRO_ACTIVATION_DELAY: "); Serial.println(_CONF.PYRO_ACTIVATION_DELAY);    
    } 
    else if(strcmp(setting, "APOGEE_DIFF_METERS") == 0) {
        _CONF.APOGEE_DIFF_METERS = atoi(value); 
        Serial.print("_CONF.APOGEE_DIFF_METERS: "); Serial.println(_CONF.APOGEE_DIFF_METERS);    
    } 
    else if(strcmp(setting, "PARACHUTE_DELAY") == 0) {
        _CONF.PARACHUTE_DELAY = atoi(value); 
        Serial.print("_CONF.PARACHUTE_DELAY: "); Serial.println(_CONF.PARACHUTE_DELAY);    
    } 
    else if(strcmp(setting, "PYRO_1_FIRE_ALTITUDE") == 0) {
        _CONF.PYRO_1_FIRE_ALTITUDE = atoi(value); 
        Serial.print("_CONF.PYRO_1_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_1_FIRE_ALTITUDE);    
    } 
    else if(strcmp(setting, "PYRO_2_FIRE_ALTITUDE") == 0) {
        _CONF.PYRO_2_FIRE_ALTITUDE = atoi(value); 
        Serial.print("_CONF.PYRO_2_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_2_FIRE_ALTITUDE);    
    } 
    else if(strcmp(setting, "PYRO_3_FIRE_ALTITUDE") == 0) {
        _CONF.PYRO_3_FIRE_ALTITUDE = atoi(value); 
        Serial.print("_CONF.PYRO_3_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_3_FIRE_ALTITUDE);    
    } 
    else if(strcmp(setting, "PYRO_4_FIRE_ALTITUDE") == 0) {
        _CONF.PYRO_4_FIRE_ALTITUDE = atoi(value); 
        Serial.print("_CONF.PYRO_4_FIRE_ALTITUDE: "); Serial.println(_CONF.PYRO_4_FIRE_ALTITUDE);    
    } 
    else if(strcmp(setting, "AUTOMATIC_ANGLE_ABORT") == 0) {
        _CONF.AUTOMATIC_ANGLE_ABORT = atoi(value); 
        Serial.print("_CONF.AUTOMATIC_ANGLE_ABORT: "); Serial.println(_CONF.AUTOMATIC_ANGLE_ABORT);    
    } 
    else if(strcmp(setting, "EXCESSIVE_ANGLE_THRESHOLD") == 0) {
        _CONF.EXCESSIVE_ANGLE_THRESHOLD = atoi(value); 
        Serial.print("_CONF.EXCESSIVE_ANGLE_THRESHOLD: "); Serial.println(_CONF.EXCESSIVE_ANGLE_THRESHOLD);    
    } 
    else if(strcmp(setting, "EXCESSIVE_ANGLE_TIME") == 0) {
        _CONF.EXCESSIVE_ANGLE_TIME = atoi(value); 
        Serial.print("_CONF.EXCESSIVE_ANGLE_TIME: "); Serial.println(_CONF.EXCESSIVE_ANGLE_TIME);    
    } 
    else if(strcmp(setting, "FIRE_PYRO") == 0) {
        DO_NOT_SAVE_FLAG = true;
        uint8_t channel = atoi(value);
        activatePyro(channel);
    } 
    else if(strcmp(setting, "RESET_PYRO") == 0) {
        DO_NOT_SAVE_FLAG = true;
        // Reset ALL pyro channels at once!
        resetPyro();
    } 
    // ----------------- GUIDANCE PAGE -----------------------------------
    else if(strcmp(setting, "PITCH_AXIS") == 0) {
         _CONF.PITCH_AXIS = atoi(value); 
        Serial.print("_CONF.PITCH_AXIS: "); Serial.println(_CONF.PITCH_AXIS); 
    } 
    else if(strcmp(setting, "YAW_AXIS") == 0) {
         _CONF.YAW_AXIS = atoi(value); 
        Serial.print("_CONF.YAW_AXIS: "); Serial.println(_CONF.YAW_AXIS); 
    } 
    else if(strcmp(setting, "ROLL_AXIS") == 0) {
         _CONF.ROLL_AXIS = atoi(value); 
        Serial.print("_CONF.ROLL_AXIS: "); Serial.println(_CONF.ROLL_AXIS); 
    } 
    else if(strcmp(setting, "GUIDING_TYPE") == 0) {
         _CONF.GUIDING_TYPE = atoi(value); 
        Serial.print("_CONF.GUIDING_TYPE: "); Serial.println(_CONF.GUIDING_TYPE); 
    } 
    else if(strcmp(setting, "ROLL_CONTROL_ENABLED") == 0) {
         _CONF.ROLL_CONTROL_ENABLED = atoi(value); 
        Serial.print("_CONF.ROLL_CONTROL_ENABLED: "); Serial.println(_CONF.ROLL_CONTROL_ENABLED); 
    } 
    else if(strcmp(setting, "ROLL_CONTROL_TYPE") == 0) {
         _CONF.ROLL_CONTROL_TYPE = atoi(value); 
        Serial.print("_CONF.ROLL_CONTROL_TYPE: "); Serial.println(_CONF.ROLL_CONTROL_TYPE); 
    } 
    else if(strcmp(setting, "SERVO_1_AXIS") == 0) {--
         _CONF.SERVO_1_AXIS = value[0]; 
        Serial.print("_CONF.SERVO_1_AXIS: "); Serial.println(_CONF.SERVO_1_AXIS); 
    } 
    else if(strcmp(setting, "SERVO_2_AXIS") == 0) {
         _CONF.SERVO_2_AXIS = value[0]; 
        Serial.print("_CONF.SERVO_2_AXIS: "); Serial.println(_CONF.SERVO_2_AXIS); 
    } 
    else if(strcmp(setting, "SERVO_3_AXIS") == 0) {
         _CONF.SERVO_3_AXIS = value[0]; 
        Serial.print("_CONF.SERVO_3_AXIS: "); Serial.println(_CONF.SERVO_3_AXIS); 
    } 
    else if(strcmp(setting, "SERVO_4_AXIS") == 0) {
         _CONF.SERVO_4_AXIS = value[0]; 
        Serial.print("_CONF.SERVO_4_AXIS: "); Serial.println(_CONF.SERVO_4_AXIS); 
    } 
    else if(strcmp(setting, "SERVO_1_OFFSET") == 0) {
         _CONF.SERVO_1_OFFSET = atoi(value); 
        Serial.print("_CONF.SERVO_1_OFFSET: "); Serial.println(_CONF.SERVO_1_OFFSET); 
    } 
    else if(strcmp(setting, "SERVO_2_OFFSET") == 0) {
         _CONF.SERVO_2_OFFSET = atoi(value); 
        Serial.print("_CONF.SERVO_2_OFFSET: "); Serial.println(_CONF.SERVO_2_OFFSET); 
    } 
    else if(strcmp(setting, "SERVO_3_OFFSET") == 0) {
         _CONF.SERVO_3_OFFSET = atoi(value); 
        Serial.print("_CONF.SERVO_3_OFFSET: "); Serial.println(_CONF.SERVO_3_OFFSET); 
    } 
    else if(strcmp(setting, "SERVO_4_OFFSET") == 0) {
         _CONF.SERVO_4_OFFSET = atoi(value); 
        Serial.print("_CONF.SERVO_4_OFFSET: "); Serial.println(_CONF.SERVO_4_OFFSET); 
    } 
    else if(strcmp(setting, "SERVO_1_ORIENTATION") == 0) {
         _CONF.SERVO_1_ORIENTATION = atoi(value); 
        Serial.print("_CONF.SERVO_1_ORIENTATION: "); Serial.println(_CONF.SERVO_1_ORIENTATION); 
    } 
    else if(strcmp(setting, "SERVO_2_ORIENTATION") == 0) {
         _CONF.SERVO_2_ORIENTATION = atoi(value); 
        Serial.print("_CONF.SERVO_2_ORIENTATION: "); Serial.println(_CONF.SERVO_2_ORIENTATION); 
    } 
    else if(strcmp(setting, "SERVO_3_ORIENTATION") == 0) {
         _CONF.SERVO_3_ORIENTATION = atoi(value); 
        Serial.print("_CONF.SERVO_3_ORIENTATION: "); Serial.println(_CONF.SERVO_3_ORIENTATION); 
    } 
    else if(strcmp(setting, "SERVO_4_ORIENTATION") == 0) {
         _CONF.SERVO_4_ORIENTATION = atoi(value); 
        Serial.print("_CONF.SERVO_4_ORIENTATION: "); Serial.println(_CONF.SERVO_4_ORIENTATION); 
    } 
    else if(strcmp(setting, "PID_PITCH_Kp") == 0) {
         _CONF.PID_PITCH_Kp = atof(value); 
        Serial.print("_CONF.PID_PITCH_Kp: "); Serial.println(_CONF.PID_PITCH_Kp); 
    } 
    else if(strcmp(setting, "PID_PITCH_Ki") == 0) {
         _CONF.PID_PITCH_Ki = atof(value); 
        Serial.print("_CONF.PID_PITCH_Ki: "); Serial.println(_CONF.PID_PITCH_Ki); 
    } 
    else if(strcmp(setting, "PID_PITCH_Kd") == 0) {
         _CONF.PID_PITCH_Kd = atof(value); 
        Serial.print("_CONF.PID_PITCH_Kd: "); Serial.println(_CONF.PID_PITCH_Kd); 
    } 
    else if(strcmp(setting, "PID_YAW_Kp") == 0) {
         _CONF.PID_YAW_Kp = atof(value); 
        Serial.print("_CONF.PID_YAW_Kp: "); Serial.println(_CONF.PID_YAW_Kp); 
    } 
    else if(strcmp(setting, "PID_YAW_Ki") == 0) {
         _CONF.PID_YAW_Ki = atof(value); 
        Serial.print("_CONF.PID_YAW_Ki: "); Serial.println(_CONF.PID_YAW_Ki); 
    } 
    else if(strcmp(setting, "PID_YAW_Kd") == 0) {
         _CONF.PID_YAW_Kd = atof(value); 
        Serial.print("_CONF.PID_YAW_Kd: "); Serial.println(_CONF.PID_YAW_Kd); 
    } 
    else if(strcmp(setting, "PID_ROLL_Kp") == 0) {
         _CONF.PID_ROLL_Kp = atof(value); 
        Serial.print("_CONF.PID_ROLL_Kp: "); Serial.println(_CONF.PID_ROLL_Kp); 
    } 
    else if(strcmp(setting, "PID_ROLL_Ki") == 0) {
         _CONF.PID_ROLL_Ki = atof(value); 
        Serial.print("_CONF.PID_ROLL_Ki: "); Serial.println(_CONF.PID_ROLL_Ki); 
    } 
    else if(strcmp(setting, "PID_ROLL_Kd") == 0) {
         _CONF.PID_ROLL_Kd = atof(value); 
        Serial.print("_CONF.PID_ROLL_Kd: "); Serial.println(_CONF.PID_ROLL_Kd); 
    } 

    // -------------------------------------------------------------------

    if(!DO_NOT_SAVE_FLAG) {
        if(!_CONF.saveConfig()) {
            Serial.println("Configuration not saved to memory!!");
        } else {
            // Emit a sound to acknolege the processing of the command
            //@TODO: Change tune to something different
                buzz(PIEZO_BUZZER, 1500, 750/12);
                buzz(PIEZO_BUZZER, 400, 1000/12);
                // buzz(PIEZO_BUZZER, 2637, 1000/12);
                // buzz(PIEZO_BUZZER, 2637, 1000/12);
                // buzz(PIEZO_BUZZER, 2637, 10000/12);
        }
    } else {
        // Emmit a sound anyways
        buzz(PIEZO_BUZZER, 1500, 750/12);
        buzz(PIEZO_BUZZER, 400, 1000/12);
    }
}

    