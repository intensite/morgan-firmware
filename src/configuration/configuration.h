#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include "Arduino.h"

#define GUIDING_TYPE_TVC 1
#define GUIDING_TYPE_FINS 2


class Configuration {

   // Private constructor, to forbid the construction from outside this class.
   Configuration ();
   //Non-copyable (such that another instance cannot be created)
   Configuration(const Configuration&);
   //Non-assignable for the same reason.
   Configuration& operator=(const Configuration&);
public:

    // PREFERENCES
    uint8_t DEBUG; // 1                                  // Set to 1 to read collected data from memory: 0 to save data to memory
    uint8_t BUZZER_ENABLE; // 0                          // Set to 1 to enable the buzzer. Set to 0 otherwise.
    uint8_t MEMORY_CARD_ENABLED; // 1                    // Set to 1 to activate the logging system.  0 to disable it (for testing)
    uint8_t DATA_RECOVERY_MODE; // 1                     // Set to 1 to read collected data from memory: 0 to save data to memory
    uint8_t FORMAT_MEMORY; // 0                          // Set to 1 to erase memory.
    uint16_t SCAN_TIME_INTERVAL; // 100                  // Speed of the control loop. Interval for sensors reading.
    
    // PYRO CONTROL
    uint8_t APOGEE_DIFF_METERS; // 10                    // Difference in meters from the appogee should trigger the pyrochanel (To prevent false appogee detection).
    uint16_t PARACHUTE_DELAY;
    uint8_t PYRO_ACTIVATION_DELAY; // 15                 // Time in seconds durring which the pyrochanel will stay on after being fired
    int16_t PYRO_1_FIRE_ALTITUDE;   // -1               // Altitude (in meters) (could be the appogee) at which the pyro chanel should be fired -1 for appogee 0 to disable.
    int16_t PYRO_2_FIRE_ALTITUDE;   // 0                // Altitude (in meters) (could be the appogee) at which the pyro chanel should be fired -1 for appogee 0 to disable.
    int16_t PYRO_3_FIRE_ALTITUDE;   // 0                // Altitude (in meters) (could be the appogee) at which the pyro chanel should be fired -1 for appogee 0 to disable.
    int16_t PYRO_4_FIRE_ALTITUDE;   // 0                // Altitude (in meters) (could be the appogee) at which the pyro chanel should be fired -1 for appogee 0 to disable.
    uint8_t AUTOMATIC_ANGLE_ABORT;   // 0                // Enable:1 or disable:0 the automatic excessive angle abort feature.
    uint8_t EXCESSIVE_ANGLE_THRESHOLD; //0               // Excessive angle threshhold for automatic abort sequence.
    int16_t EXCESSIVE_ANGLE_TIME; //0                    // Time (in millisec) delay at Excessive angle condition after which automatic abort sequence will be triggered.

    // GUIDANCE CONTROL
    uint8_t GUIDING_TYPE;                              // Type of guiding system (1=TVC, 2=Fins)
    uint8_t ROLL_CONTROL_ENABLED;                      // 1=Enabled, 0=Disabled
    uint8_t ROLL_CONTROL_TYPE;                         // If enabled, what type of roll control (1=Reaction wheel 2=Fins on the X-Axis)

    // SERVO AXIS MAPPING  (POSSIBLE VALUES: X,Y,Z,A)   // A Stands for Other Possibly for reaction wheel
    char SERVO_1_AXIS;                                   // Two Servo can be on the same Axis
    char SERVO_2_AXIS;
    char SERVO_3_AXIS;
    char SERVO_4_AXIS;

    int8_t SERVO_1_OFFSET;                              // Used to fine tune the SERVOs alingments
    int8_t SERVO_2_OFFSET;                             // Used to fine tune the SERVOs alingments
    int8_t SERVO_3_OFFSET;                              // Used to fine tune the SERVOs alingments
    int8_t SERVO_4_OFFSET;                             // Used to fine tune the SERVOs alingments
    int8_t SERVO_1_ORIENTATION;                        // Used to reverse the servo rotation direction possible values (1, -1)
    int8_t SERVO_2_ORIENTATION;                        // Used to reverse the servo rotation direction possible values (1, -1)
    int8_t SERVO_3_ORIENTATION;                        // Used to reverse the servo rotation direction possible values (1, -1)
    int8_t SERVO_4_ORIENTATION;                        // Used to reverse the servo rotation direction possible values (1, -1)
    int8_t MAX_FINS_TRAVEL;

   // PID TUNING
    float PID_PITCH_Kp;
    float PID_PITCH_Ki;
    float PID_PITCH_Kd;
    float PID_YAW_Kp;
    float PID_YAW_Ki;
    float PID_YAW_Kd;
    float PID_ROLL_Kp;
    float PID_ROLL_Ki;
    float PID_ROLL_Kd;

   // IMU AXIS PHYSICAL LOCATION (ypr[3])
    uint8_t PITCH_AXIS;
    uint8_t YAW_AXIS;
    uint8_t ROLL_AXIS;
   
    // IMU CALIBRATION
    int16_t X_GYRO_OFFSETS;
    int16_t Y_GYRO_OFFSETS;
    int16_t Z_GYRO_OFFSETS;
    int16_t X_ACCEL_OFFSETS;
    int16_t Y_ACCEL_OFFSETS;
    int16_t Z_ACCEL_OFFSETS;

   // TEST RELATED CONSTANTS
   uint8_t PYRO_CHANNELS_TEST;    // Bit field       8,4,2,1   1=1, 2=2, 3=4, 4=8  1&2=3, etc..
   
   //Static member function that returns the instance of the singleton by reference.
   static Configuration& instance(); 
   bool readConfig();
   int saveConfig();
};

#define _CONF Configuration::instance()
#endif

