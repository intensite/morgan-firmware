#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H
// #define I2CDEV_IMPLEMENTATION I2CDEV_BUILTIN_FASTWIRE

// PIN ASSIGNMENT 
#define FINS_SERVO_1_PIN 16
#define FINS_SERVO_2_PIN 17
#define FINS_SERVO_3_PIN 23         // Used to be GPIO5 but somehow it didn't work (Board had to be modified)
#define FINS_SERVO_4_PIN 18

#define PARACHUTE_IGNITER_PIN 32           // To Be Removed
#define PYRO_CHANEL_1 32
#define PYRO_CHANEL_2 33
#define PYRO_CHANEL_3 25
#define PYRO_CHANEL_4 26                   // Shared Pin (PY4-HDR2)

#define R_LED 4                            // Analog pin for the Red LED
#define B_LED 2                            // Analog pin for the Blue LED
#define G_LED 0                            // Analog pin for the Green LED
#define PIEZO_BUZZER 27                    // Analog pin for the Green LED
#define REMOVE_BEFORE_FLIGHT 23            // HEADER1  // HIGH IF READY TO FLY. (Pin is configured as INPUT_PULLUP ) 
#define HEADER1 23                         // HEADER1  
#define VOLTAGE 34                         // VOLTAGE  


// #define SDA 21 
// #define SCL 22 

// // ALTITUDE & ANGLE VARIABLES
// #define PITCH_AXIS 2                        // MPU-6050 Axis when mounted on rocket configuration (Y Axis when flat)
// #define YAW_AXIS 0                          // MPU-6050 Axis when mounted on rocket configuration (Z Axis when flat)
// #define ROLL_AXIS 1                         // MPU-6050 Axis when mounted on rocket configuration (X Axis when flat)

// #define APOGEE_DIFF_METERS 10               // Used to specify minimum altitude for liftoff and minimum decent for parachute deployment. 
// #define EXCESSIVE_ANGLE_THRESHOLD 50        // Used to specify maximum angle before abort sequence is initiated.
// #define SCAN_TIME_INTERVAL 100              // Used to specify the refresh rate in mili-seconds of the instruments (altimeter and gyroscope).
// #define PARACHUTE_DELAY 1500                // Time (in milli sec.) to wait after disabling servos before the parachute is deployed. 
//                                             // This delay is to maximize the battery power before ignition.
//                                             // WARNING: This is a blocking delay. Nothing else can be done, No other reading of sensors, etc will happen.


// // SERVO STUFF
// #define SERVO_1_OFFSET -11                  // Used to compensate the servo #1 (Pitch) misalignment
// #define SERVO_2_OFFSET -2                   // Used to compensate the servo #1 (Roll) misalignment
// #define SERVO_1_ORIENTATION -1              // Used to reverse the orientation of the servo #1 (possible values 1, -1) 
// #define SERVO_2_ORIENTATION -1              // Used to reverse the orientation of the servo #2 (possible values 1, -1)

// #define MAX_FINS_TRAVEL 15                  // Used to specify limits of the fins travel in degrees (+/-)


// // PITCH PID CONSTANTS
// #define PID_PITCH_Kp 2
// #define PID_PITCH_Ki 0
// #define PID_PITCH_Kd 0.5

// // ROLL PID CONSTANTS
// #define PID_ROLL_Kp 2
// #define PID_ROLL_Ki 0
// #define PID_ROLL_Kd 0.5

// CALIBRATION CONSTANTS CHANGE TO CALIBRATE
// #define X_GYRO_OFFSETS 5
// #define Y_GYRO_OFFSETS 41
// #define Z_GYRO_OFFSETS 57
// #define X_ACCEL_OFFSETS -1198
// #define Y_ACCEL_OFFSETS 97
// #define Z_ACCEL_OFFSETS 1752

// #define X_GYRO_OFFSETS 24
// #define Y_GYRO_OFFSETS 43
// #define Z_GYRO_OFFSETS 525
// #define X_ACCEL_OFFSETS -1109
// #define Y_ACCEL_OFFSETS 841
// #define Z_ACCEL_OFFSETS 525



// #define DEBUG 1                             // Set to 1 to display debug info to the serial console. Set to 0 otherwise.
// #define BUZZER_ENABLE 0                     // Set to 1 to enable the buzzer. Set to 0 otherwise.
// #define MEMORY_CARD_ENABLED 1               // Set to 1 to activate the logging system.  0 to disable it (for testing)
// #define DATA_RECOVERY_MODE 1                // Set to 1 to read collected data from memory: 0 to save data to memory
// #define FORMAT_MEMORY 0                     // Set to 1 to erase memory.

#endif // CONFIG_FILE_H

