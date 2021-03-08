#define ESP32_RTOS  // Uncomment this line if you want to use the code with freertos only on the ESP32
                    // Has to be done before including "OTA.h"

#include "./ota/OTA.h"

#include <Arduino.h>
#include "./config.h"
#include "./global.h"
#include "./gyro/gyro.h"
#include "./altimeter/altitude.h"
#include "./servo/fins_servo.h"
#include "./flight_correct/correct.h"
#include "./buzzer/buzzer.h"
#include "./led_color/led_color.h"
#include "./storage/Storage.h"
#include "./storage/LogSystem.h"
#include "./configuration/configuration.h"
#include "./data/data.h"
#include "./radio/radio.h"
// #include "./storage/LogSystem_SD.h"
#include "./parachute/parachute.h"
#include <Wire.h>
// #include "./bluetooth/bluetooth.h"
#include "./websocket/ws.h"
#include "./command/command.h"
#include "./lib/TaskScheduler.h"
#include "./lib/TaskScheduler.h"
#include "CREDENTIALS"


enum State_enum {LAUNCHPAD, ARMED, THRUST_ST1, COASTING_INTER, THRUST_ST2, COASTING, DESCENT, CHUTE_DESCENT, LANDED };
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool is_abort = false;
bool is_parachute_deployed = false;
int16_t g_servo_pitch = 0;
int16_t g_servo_yaw = 0;
int16_t g_servo_roll = 0;
// byte IS_READY_TO_FLY;   // Used with the REMOVE_BEFORE_FLIGHT jumper

// const long interval = 100;
unsigned long previousMillis = 0;
unsigned long previousHBeatMillis = 0;
unsigned long previousLaunchMillis = 0;   // Used for launch detection
unsigned long previousBurnoutMillis = 0;   // Used for burnout detection

bool setup_error = false;
State_enum currentState;

Altitude altitude;
Gyro gyro;
float voltage = 0;
CliCommand cli;
bool ledStatus;
// Scheduler
Scheduler ts;
// Task (object) for second Core (Used to run radio and GPS stuff)
TaskHandle_t TaskCore0;


void flashLEDcb();
void beepSequencecb();
void updateBLEdiags_cb();
void updateBLEparams_cb();
void updateBLEGuidanceConfig_cb();
void measureVoltage_cb();
void abort_flight();
bool detectLiftoff();
void state_LAUNCHPAD();
void state_ARMED();
void state_THRUST_ST1();
void state_COASTING_INTER();
void state_THRUST_ST2();
void state_COASTING();
void state_DESCENT();
void state_CHUTE_DESCENT();
void state_LANDED();
void displaySensorData();
void loopCore0(void *);
void calibrate();

// ================================================================
// Tasks definition
// ================================================================
Task tflashLED ( 2 * TASK_SECOND, -1, &flashLEDcb, &ts, true );
Task tbeepSequence ( 2 * TASK_SECOND, -1, &beepSequencecb, &ts, true );
Task tMeasureVoltage ( 10 * TASK_SECOND, -1, &measureVoltage_cb, &ts, true );
Task tupdateBLEdiags ( 200 * TASK_MILLISECOND, -1, &updateBLEdiags_cb, &ts, true );     // BLE Diagnostics (fast refresh)
Task tupdateBLEparams ( 500 * TASK_MILLISECOND, -1, &updateBLEparams_cb, &ts, true );   // BLE Params (slow refresh)
Task tupdateBLEGuidanceConfig ( 550 * TASK_MILLISECOND, -1, &updateBLEGuidanceConfig_cb, &ts, true );   // BLE guidance Params (slow refresh)

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define MPU_INTERRUPT_PIN 35  // use pin 3 as pin 2 interfere with servo.

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() {
    portENTER_CRITICAL_ISR(&mux);       // Not sure if this mutex stuff brings anithing...
    mpuInterrupt = true;
    portEXIT_CRITICAL_ISR(&mux);        // Not sure if this mutex stuff brings anithing...
}
 



void testSequence() {

    Serial.println(F("Begin of tests..........."));
    Serial.println(F("Testing servos..."));
    testServo();

    Serial.println(F("Testing LED and Buzzer..."));

    // Test LED 
    led_color(LED_COLOR_RED);
    delay(500);               
    led_color(LED_COLOR_GREEN);
    delay(500);               
    led_color(LED_COLOR_BLUE);
    delay(500);               
    led_color(LED_COLOR_OFF);

    // Test PIEZO_BUZZER 
    const int8_t tone_delay = 83;  //1000/12
    buzz(PIEZO_BUZZER, 2637, tone_delay);
    buzz(PIEZO_BUZZER, 3136, tone_delay);
    buzz(PIEZO_BUZZER, 2093, tone_delay);
    buzz(PIEZO_BUZZER, 0, tone_delay);
    Serial.println(F("End of tests..........."));
}


// ================================================================
// ===      persistData: store flight data to flash memory     ===
// ================================================================
int8_t persistData() {

    if(_CONF.MEMORY_CARD_ENABLED == 0 ) {
        return 0;
    }

    if(gyro.ypr[1] == 0 || gyro.ypr[2] ==0) {
         Serial.println("Data invalid do nothing");
        // Data invalid do nothing
        return 1;
    }

    lr::LogRecord logRecord(
        millis(), 
        currentState,
        altitude.current_altitude, 
        // (int16_t) (gyro.ypr[_CONF.PITCH_AXIS] * 180/M_PI),  // Pitch: Must be improved
        // (int16_t) (gyro.ypr[_CONF.ROLL_AXIS] * 180/M_PI),  // Roll:  Must be improved
        (int16_t) (gyro.ypr[_CONF.PITCH_AXIS]),  // Pitch: Must be improved
        (int16_t) (gyro.ypr[_CONF.ROLL_AXIS]),  // Roll:  Must be improved
        g_servo_pitch, // Servo Pitch: ToDo
        g_servo_roll, // Servo Roll : ToDo
        is_parachute_deployed, 
        is_abort, 
        altitude.temperature, // Temperature
        voltage, // Batt (a faire)
        gyro.z_gforce  // gForces
    );
    if (!lr::LogSystem::appendRecord(logRecord)) {
        Serial.println("Probleme de storrage: verifier memoire pleine");
        return 0;
    } else {
        // Serial.println("Record saved: ");
    }
    return 1;
}


// ================================================================
// ===               MAIN SETUP ROUTINE                         ===
// ================================================================
void setup() {

    // Remap the hardware serial ports so we can use the default Rx/Tx pins for Serial 2
    // HardwareSerial Serial(2);
    // HardwareSerial Serial2(0);


    // initialize serial communication
    Serial.begin(115200);
    Serial.println("Booting");

    if(USE_OTA_UPDATE)
        setupOTA("Morgan", SSID, PASSWORD);

    is_abort = false;
    is_parachute_deployed = false;
    g_servo_pitch = 0;
    g_servo_roll = 0;
    currentState = LAUNCHPAD;

    // WiFi.mode(WIFI_OFF);


    delay(500);                 // waits for the serial to settle

    _CONF.readConfig();         // Read the config from SPIFFS Memory
    
    pinMode(R_LED, OUTPUT);     digitalWrite(R_LED, HIGH);
    pinMode(G_LED, OUTPUT);     digitalWrite(G_LED, HIGH);
    pinMode(B_LED, OUTPUT);     digitalWrite(B_LED, HIGH);
    led_color(LED_COLOR_BLUE);    // Set the LED to blue durring setup

    pinMode(PIEZO_BUZZER, OUTPUT);
    pinMode(PARACHUTE_IGNITER_PIN, OUTPUT); digitalWrite(PARACHUTE_IGNITER_PIN, LOW);
    pinMode(LAUCH_DETECTION_PIN, INPUT_PULLUP);

    //************************************************************************************
    // RESET MEMORY VARIABLES  (Now set from bluetooth)
    //************************************************************************************
    _CONF.FORMAT_MEMORY = 0;
    _CONF.MEMORY_CARD_ENABLED = 0;
    _CONF.DATA_RECOVERY_MODE = 0;

    // pinMode(REMOVE_BEFORE_FLIGHT, INPUT_PULLUP); // HIGH IF READY TO FLY. Used with Jumper (Pin is configured as INPUT_PULLUP  ) 
    // IS_READY_TO_FLY = digitalRead(REMOVE_BEFORE_FLIGHT); 
    // Serial.println(F("=============================================="));
    // Serial.print(F("IS_READY_TO_FLY: ")); Serial.println(IS_READY_TO_FLY);
    // Serial.println(F("=============================================="));

    // Attach the interrupt pin
    pinMode(MPU_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    
    ledStatus = LOW;

    // Setup bluetooth
    // setupBLE(cli);
    setupWebSocket(cli);

    setupServo();

    calibrate();
    
    // ================================================================
    // ===               Storage system initialization              ===
    // ================================================================
    Serial.println(F("Initialize the log system"));
    Serial.print("_CONF.MEMORY_CARD_ENABLED : "); Serial.println(_CONF.MEMORY_CARD_ENABLED);
    
    delay(2000);
    // FLASH LOG SYSTEM
    if (!lr::Storage::begin()) {
        Serial.println("Storage Problem");
        is_abort = true;
        return;
    } else {
        lr::LogSystem::begin(0);  
        Serial.println("Storage seems OK");
    }
    // End of Storage system initialization
    // ================================================================    

    testSequence();
    
    // if(_CONF.DEBUG && IS_READY_TO_FLY) {
    //     debugParachute();  // WARNING TEST ONLY! REMOVE THIS LINE BEFORE FLIGHT.
    // }

    // Setup radio(nRF24) and GPS
    setupRadio();

    // Create a task to process slow radio and GPS stuff on the second core of the ESP32
    // Pin it to CORE-0
    xTaskCreatePinnedToCore(
        loopCore0, /* Function to implement the task */
        "TaskCore0", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &TaskCore0,  /* Task handle. */
        0); /* Core where the task should run */

}

void calibrate() {
     Serial.print("CALIBRATION TRIGGERED!! ");
     Serial.println(_CONF.LOCAL_KPA);

    if (gyro.setupGyro() != 0) {
        setup_error = true;
        // LED RED
        led_color(LED_COLOR_RED);
        is_abort = true;
        Serial.println(F("Problem with Gyroscope not detected..."));
        return;
    }

    if (altitude.setupAlti() !=0) {
        setup_error = true;
        // LED RED
        led_color(LED_COLOR_RED);
        is_abort = true;
        Serial.println(F("Problem with altitmeter not detected..."));
        return;
    }

}

// ================================================================
// ===               Main loop                                  ===
// ================================================================
void loop() {

    unsigned long currentMillis = millis();

    // The abort sequence was triggered (throw your arms in the air) exit the main loop
    // This is avaluated first
    if (is_abort) {
        abort_flight();
        return;
    }
    
    // Some data from the Gyroscope is ready to be processed
    // This and altitude data need to be done for all states so it is done first and outside ot the state evaluation
    if(mpuInterrupt) {
        portENTER_CRITICAL(&mux);       
        mpuInterrupt = false;
        portEXIT_CRITICAL(&mux);

        gyro.ProcessGyroData();
    }

    // Read the altitude
    altitude.processAltiData();

    // Used to slowdown the process to the number of milliseconds defined in variable interval (see config.h file)
    if (currentMillis - previousMillis >= _CONF.SCAN_TIME_INTERVAL) {
        previousMillis = currentMillis; 

        //****************************************
        // WARNING!!
        // Manual Mode Overide
        if(_CONF.MANUAL_STATE != -1) {
            currentState = (State_enum)_CONF.MANUAL_STATE;
            _CONF.MANUAL_STATE = -1;    // Reset MANUAL_STATE
        }
        //****************************************

        //enum State_enum {LAUNCHPAD, ARMED, THRUST_ST1, THRUST_ST2, COASTING, DESCENT, CHUTE_DESCENT, LANDED };
        switch(currentState) {
            case LAUNCHPAD:
                state_LAUNCHPAD();
                break;
            case ARMED:
                state_ARMED();
                break;
            case THRUST_ST1:
                state_THRUST_ST1();
                break;
            case COASTING_INTER:
                state_COASTING_INTER();
                break;
            case THRUST_ST2:
                state_THRUST_ST2();
                break;
            case COASTING:
                state_COASTING();
                break;
            case DESCENT:
                state_DESCENT();
                break;
            case CHUTE_DESCENT:
                state_CHUTE_DESCENT();
                break;
            case LANDED:
                state_LANDED();
                break;
        }

    }
   

    // Debug stuff (Make sure to disable before flight)
    if (_CONF.DEBUG) 
        //displaySensorData();  // Output sensors data to serial console.  Enabled only in DEBUG Mode to maximize computer performances.

    // Persist flight data to memory 
    //@TODO:  Decide if data persistance should be moved in a state function or not
    if (_CONF.MEMORY_CARD_ENABLED) {
        persistData();
    }

    // Execute all the enabled tasks according to their respective schedules 
    ts.execute();
}



// Paralel loop to process slow radio and GPS stuff on the second core of the ESP32
// Pin it to CORE-0
void loopCore0(void * parameter) {
    // It must run forever, so this is the construct
    for (;;) {
        updateRadioGPS();
    }
}


/****************************************************************************************************************************
 *                                      TASK CALLBACKs
 *  Used by the task scheduler define at the top of the file
 *  The original function was blinking pulsating the led.  This function should be pin to the second core.
 ****************************************************************************************************************************/
void flashLEDcb() {

    // Called by task tflashLED
    // Flash Green LED every seconds
    if(ledStatus == LOW) {
        ledStatus = HIGH;
        led_color(LED_COLOR_OFF);
    } else {
        ledStatus = LOW;
        if(!_CONF.MEMORY_CARD_ENABLED) {  
            // If the Memory card is not recording, flash blue
            // led_color(LED_COLOR_BLUE);
            // delay(50);
            // led_color(LED_COLOR_OFF);
            // delay(100);
            // led_color(LED_COLOR_BLUE);
            // delay(50);
            // led_color(LED_COLOR_OFF);
            // delay(100);

            if( currentState == ARMED ) {
                led_color(LED_COLOR_PURPLE);
            } else {
                led_color(LED_COLOR_BLUE);
            }
        }else {
            // led_color(LED_COLOR_GREEN);
            // delay(50);
            // led_color(LED_COLOR_OFF);
            // delay(100);
            // led_color(LED_COLOR_GREEN);
            // delay(50);
            // led_color(LED_COLOR_OFF);
            // delay(100);
            if( currentState == ARMED ) {
                led_color(LED_COLOR_PURPLE);
            } else {
                led_color(LED_COLOR_GREEN);
            }
        }
    }

    

}

void beepSequencecb() {
    // Called by task tbeepSequence
    if(_CONF.BUZZER_ENABLE) {
        buzz(PIEZO_BUZZER, 2637, 1000/12);
        buzz(PIEZO_BUZZER, 3136, 1000/12);
        buzz(PIEZO_BUZZER, 2093, 1000/12);
    }   
}

void updateBLEdiags_cb() {  // Updated on a fast schedule
    // Called by task updateBLEdiags_cb

    if(!_CONF.DATA_RECOVERY_MODE) {
        // updateDiagnostics(gyro.ypr, gyro.aaWorld.x, gyro.aaWorld.y, gyro.aaWorld.z, altitude.current_altitude, altitude.temperature, 
        //         altitude.pressure, altitude.humidity, voltage);
        updateDiagnostics(gyro.ypr, gyro.accel[0], gyro.accel[1], gyro.accel[2], altitude.current_altitude, altitude.temperature, 
                altitude.pressure, altitude.humidity, voltage, currentState);
    }
}
void updateBLEparams_cb() {  // Updated on a slower schedule
    // Called by task updateBLEparas_cb
    if(!_CONF.DATA_RECOVERY_MODE) {
        updateBLEparams();
    }
}
void updateBLEGuidanceConfig_cb() {  // Updated on a slower schedule
    // Called by task tupdateBLEGuidanceConfig
    if(!_CONF.DATA_RECOVERY_MODE) {
        updateGuiding();
    }
}

void measureVoltage_cb() {
        /* Voltage mesurement */
        // int8_t temp;
        voltage = (float)analogRead(VOLTAGE);// / 4096 * 38.846;
        // Serial.println(voltage);
        voltage = (float)(voltage / 4096 * 50);

        // Debug
        // temp = (int8_t)(voltage * 10 + .5);
        //Serial.print("Voltage: "); Serial.println((float)(temp/10));
}

bool detectLiftoff() {  //@TODO:  To be completely rewriten using mechanical switch as accelerometer wasn't reliable
    /** 
     * To be used only on the LAUNCHPAD state
     * Using the accelerometer, if the accelleration on the Z axis is >= than Threshold (Use a constant here)
     * for more than 0.1 Second (Use another contant here)
     * return true, else return false
     */

    // Old accelerometer method
    // if(abs(gyro.z_gforce) >= 1.5 ) {
    //     if (previousLaunchMillis == 0) {
    //         previousLaunchMillis = millis();
    //     }

    //     if((millis() - previousLaunchMillis) >= 100) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // } else {
    //     previousLaunchMillis = 0;
    //     return false;
    // }

    if(digitalRead(LAUCH_DETECTION_PIN) == HIGH ) {
        if (previousLaunchMillis == 0) {
            previousLaunchMillis = millis();
        }

        if((millis() - previousLaunchMillis) >= 100) {
            return true;
        } else {
            return false;
        }
    } else {
        previousLaunchMillis = 0;
        return false;
    }
}

bool detectBurnout() {
    /** 
     * Used durring powered flight stages to detect when engins burnout
     * Using the accelerometer, if the accelleration on the Z axis is <= than Threshold (Use a constant here)
     * for more than 0.1 Second (Use another contant here)
     * return true, else return false
     */
    
    // IF Accel Z <= 0.2M/s2 for more than 0.1 Sec   ===> MECO

    if((abs(gyro.z_gforce) * 9.80665) - 9.80665 <= 0.2 ) {
        if (previousBurnoutMillis == 0) {
            previousBurnoutMillis = millis();
        }

        if((millis() - previousBurnoutMillis) >= BURNOUT_TIMER) {
            return true;
        } else {
            return false;
        }
    } else {
        previousBurnoutMillis = 0;
        return false;
    }

}

void abort_flight() {
    Serial.println("Abort sequence triggered"); 

    // LED RED
    digitalWrite(R_LED, LOW);
    digitalWrite(G_LED, HIGH);
    digitalWrite(B_LED, HIGH);
}

bool detectEngineThrust() {
    
}
bool detectTouchDown() {

}

/****************************************************************************************************************************
 *                                      State worker functiond
 *                            Called by the main loop depending on states
 ****************************************************************************************************************************/
void state_LAUNCHPAD() {

    //Serial.println(F("state_LAUNCHPAD"));
    // Disable beeps until ARMED and keep LED
    tbeepSequence.disable();

    // Calibrate sensors
    if(_CONF.CALIBRATE) {
        _CONF.CALIBRATE = false;
        calibrate();
    }

    // Memory format 
    if(_CONF.FORMAT_MEMORY) {
        _CONF.FORMAT_MEMORY = 0;
        _CONF.MEMORY_CARD_ENABLED = 0;
        Serial.println(F("**** Erassing memory....This takes a while...."));
        lr::LogSystem::format();
        delay(20000); // 20 Sec. seems to do the job just fine
        Serial.print("Is flash chip busy? : "); Serial.println(lr::LogSystem::isBusy());
    }

    // Recover data from memory (Should maybe be moved to a pre-flight stage)
    if (_CONF.DATA_RECOVERY_MODE) {
        _CONF.DATA_RECOVERY_MODE = 0;
        readDataToSerial();
        // readDataToBLE();  //@TODO: Change for Wifi transmission of flight data
    }

    // Handle communication with outside world
    processWebSocket();
    cli.handleSerial();

    //@TODO:  Detect ARMING SIGNAL!
    // From mission control
    if(_CONF.ARMED_STATUS == 1) {
        // Chage state to ARMED
        Serial.println("Rocket is now ARMED!");
        tbeepSequence.enable();  // Enable the beep signal since we are armed
        currentState = ARMED;
    }

    //DEBUG ONLY REMOVE BEFORE FLIGHT
    //processTrajectory(gyro.ypr); //DEBUG ONLY REMOVE BEFORE FLIGHT

}

void state_ARMED() {

    //Serial.println(F("state_ARMED"));
    // Handle communication with outside world
    processWebSocket();
    cli.handleSerial();

    // From mission control, process disarming signal
    if(_CONF.ARMED_STATUS == 0) {
        // Chage state to DISARMED
        Serial.println("Rocket is now DISARMED!");
        currentState = LAUNCHPAD;
    }

    //@TODO:  Perform last calibration before launch here
    // Only once!

    

    if (detectLiftoff()) {
        Serial.println("Lift-off detected");

        // Disable un-necessary tasks
        tflashLED.disable();
        tbeepSequence.disable();
        // tupdateBLEdiags.disable();
        // tupdateBLEparams.disable();
        // tupdateBLEGuidanceConfig.disable();

        // Chage state to Powered Flight
        Serial.println("Chage state to THRUST_ST1");
        currentState = THRUST_ST1;
    }  

}


void state_THRUST_ST1() {
    
    //Serial.println(F("state_THRUST_ST1"));
    // debug only, disable in flight condition
    // Handle communication with outside world 
    if (_CONF.DEBUG || _CONF.MANUAL_STATE != -1) {
        processWebSocket();
        cli.handleSerial();
    }

    // Process trajectory with the servos
    processTrajectory(gyro.ypr);

    if (detectBurnout()) {

        if (DOUBLE_STAGE_CONFIG == 1) {
            // Chage state to Coasting Inter Stage
            Serial.println("Chage state to COASTING_INTER");
            currentState = COASTING_INTER;
        } else {
            Serial.println("Chage state to COASTING");
            currentState = COASTING;
        }
    }
    
    //  Check for apogee even during the first power stage
    if(altitude.detectApogee()) {
        // Chage state to Descent
        Serial.println("Chage state to DESCENT");
        currentState = DESCENT;
    }
}

void state_COASTING_INTER() { 
    //Serial.println(F("state_COASTING_INTER"));

    // debug only, disable in flight condition
    // Handle communication with outside world 
    if (_CONF.DEBUG || _CONF.MANUAL_STATE != -1) {
        processWebSocket();
        cli.handleSerial();
        tbeepSequence.disable();
    }

    if (DOUBLE_STAGE_CONFIG == 1) {
        if (detectLiftoff()) {
                // Detect the thrust of the second stage
                Serial.println("Acceleration detected");
                Serial.println("Chage state to THRUST_ST2");
                currentState = THRUST_ST2;
        }
    }

    if(altitude.detectApogee()) {
        // Chage state to Descent
         Serial.println("Chage state to DESCENT");
        currentState = DESCENT;
    }
}

void state_THRUST_ST2() {

    //Serial.println(F("state_THRUST_ST2"));
    
    // debug only, disable in flight condition
    // Handle communication with outside world 
    if (_CONF.DEBUG || _CONF.MANUAL_STATE != -1) {
        processWebSocket();
        cli.handleSerial();
        tbeepSequence.disable();
    }

    // Process trajectory with the servos
    processTrajectory(gyro.ypr);

    if (detectBurnout()) {
        // Chage state to final Coasting
         Serial.println("Chage state to COASTING");
        currentState = COASTING;
    }
    
    //  Check for apogee even during the second power stage
    if(altitude.detectApogee()) {
        // Chage state to Descent
        Serial.println("Chage state to DESCENT");
        currentState = DESCENT;
    }
}

void state_COASTING() { 
    // Final Coasting stage before apogee
    //Serial.println(F("state_COASTING"));

    // debug only, disable in flight condition
    // Handle communication with outside world 
    if (_CONF.DEBUG || _CONF.MANUAL_STATE != -1) {
        processWebSocket();
        cli.handleSerial();
        tbeepSequence.disable();
    }

    if(altitude.detectApogee()) {
        // Chage state to uncontroled descent
        Serial.println("Chage state to DESCENT");
        currentState = DESCENT;
    }
}

void state_DESCENT() { 

    //Serial.println(F("state_DESCENT"));

    // debug only, disable in flight condition
    // Handle communication with outside world 
    if (_CONF.DEBUG || _CONF.MANUAL_STATE != -1) {
        processWebSocket();
        cli.handleSerial();
        tbeepSequence.disable();
    }

    //Detect Parachute Altitude
    if(altitude.detectChuteAltitude()) {
        // Chage state to PARACHUTE CONTROLED DESCENT
        Serial.println("Chage state to CHUTE_DESCENT");
        currentState = CHUTE_DESCENT;
    }
}

void state_CHUTE_DESCENT() { 

    //Serial.println(F("state_CHUTE_DESCENT"));
    // debug only, disable in flight condition
    // Handle communication with outside world 
    if (_CONF.DEBUG || _CONF.MANUAL_STATE != -1) {
        processWebSocket();
        cli.handleSerial();
        tbeepSequence.disable();
    }

    if(altitude.detectTouchDown()) {
        Serial.println("Chage state to LANDED");
        // Chage state to LANDED
        currentState = LANDED;
    }

}
void state_LANDED() { 
    //Serial.println(F("state_LANDED"));
    processWebSocket();
    cli.handleSerial();

    tbeepSequence.enable();  // Enable the beep signal to help find the landed rocket

}

/*****************************************************************
 *  Used to display sensor data onthe cosole. Mainly to debug.
 *  Should no be used durring flight as it disturbs the interupts of the I2C
 * ***************************************************************/
void displaySensorData() {
    
        // Debug stuff
        // Serial.print(F("Pitch:"));
        // Serial.print((int16_t)(gyro.ypr[_CONF.PITCH_AXIS] * 180/M_PI));
        // Serial.print(F("\tYaw:"));
        // Serial.print((int16_t)(gyro.ypr[_CONF.YAW_AXIS] * 180/M_PI));
        // Serial.print(F("\tRoll:"));
        // Serial.print((int16_t)(gyro.ypr[_CONF.ROLL_AXIS] * 180/M_PI));
        
        // Serial.print(F("Accel_X:"));
        // Serial.print(gyro.accel[0]);
        // Serial.print(F("Accel_Y:"));
        // Serial.print(gyro.accel[1]);
        // Serial.print(F("Accel_Z:"));
        // Serial.print(gyro.accel[2]);
        


        // Serial.print(F("\tAltitude:"));
        // Serial.print(altitude.current_altitude);

        // Serial.print(F("\tTemperature:"));
        // Serial.println(altitude.temperature);

        // Serial.print(F("\tServo Yaw & Pitch: "));Serial.print(g_servo_yaw);Serial.print(", ");Serial.println(g_servo_pitch);

        // //***************************************************************************************
        // // Plotter stuff
        // //***************************************************************************************
        //Serial.print(F("Pitch:"));
        Serial.print((int16_t)(gyro.ypr[_CONF.PITCH_AXIS])); Serial.print(",");
        //Serial.print(F("\tYaw:"));
        Serial.print((int16_t)(gyro.ypr[_CONF.YAW_AXIS])); Serial.print(",");
        //Serial.print(F("\tRoll:"));
        Serial.print((int16_t)(gyro.ypr[_CONF.ROLL_AXIS] )); Serial.print(",");
        //Serial.print(F("\tAltitude:"));
        Serial.print(altitude.current_altitude); Serial.print(",");
        Serial.print(gyro.z_gforce); Serial.print(",");
        //Serial.print(F("\tTemperature:"));
        Serial.println(altitude.temperature); 
        // //***************************************************************************************

        // /* Voltage mesurement */
        // float temp;
        // voltage = (float)analogRead(A3);// / 4096 * 38.846;
        // // Serial.println(voltage);
        // voltage = (float)(voltage / 4096 * 40.125);
        // temp = (int8_t)(voltage * 10 + .5);
        // Serial.println((float)(temp/10));

        // Serial.print(F("\Voltage:"));
        // Serial.println(voltage);

} 


void debugParachute() {

    // // Disable the test if the REMOVE_BEFORE_FLY jumper is not present 
    // if (IS_READY_TO_FLY == LOW)
    //     return;

    // int16_t countdown = 10;
    // // Serial.println("Debug mode. Press any key to deploy parachute....................");
    // // while(Serial.available() == 0) { }  // There really is nothing between the {} braces
    // // char x = Serial.read();

    // while(countdown >=0) {
    //     delay(1000);
    //     buzz(PIEZO_BUZZER, 2637, 1000/12);
    //     countdown--;
    // }

    // buzz(PIEZO_BUZZER, 2637, 1000/12);
    // buzz(PIEZO_BUZZER, 2637, 1000/12);
    // buzz(PIEZO_BUZZER, 2637, 1000/12);
    // buzz(PIEZO_BUZZER, 2637, 1000/12);
    // buzz(PIEZO_BUZZER, 2637, 10000/12);
    // deployParachute();
    // is_abort = true;
    // is_parachute_deployed = true;
    // return;
}