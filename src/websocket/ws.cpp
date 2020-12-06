#include "Arduino.h"
#include <WebSocketsServer.h>
#include <WiFi.h>
#include "../configuration/configuration.h"
#include "../command/command.h"
#include "./ws.h"
#include "../lib/PString.h"
#include "../lib/Streaming.h"
#include "CREDENTIALS"


// Constants
const char *ssid = "ESP32-AP";
const char *password =  "morgan123";
const int dns_port = 53;
const int ws_port = 1337;
uint8_t _client_num = NULL;
bool deviceConnected = false;

// WebSocketsServer webSocket; = WebSocketsServer(1337);
WebSocketsServer webSocket(ws_port);
CliCommand clii; // Passed from the setupBLE function to process the received commands

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      {
        Serial.printf("[%u] Disconnected!\n", client_num);
        _client_num = NULL;  
        deviceConnected = false;  
      }
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
        _client_num = client_num;
        deviceConnected = true;
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);

    //   clii.handleReceivedMessage((const char*)payload);
      clii.handleReceivedMessage((char*)payload);
       break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}


void setupWebSocket(CliCommand& cliPtr) {

    if(USE_BUILT_IN_AP) {
        //************BUILT-IN AP SECTION ********************************************
          // Start access point
          WiFi.softAP(ssid, password);

          ////Print our IP address
          Serial.println();
          Serial.println("AP running");
          Serial.print("My IP address: ");
          Serial.println(WiFi.softAPIP());
    } else {
        //************EXISTING AP SECTION ********************************************
          WiFi.begin(SSID, PASSWORD);
          
          while (WiFi.status() != WL_CONNECTED) 
          {
              Serial.print(".");
          }

          Serial.println();
          Serial.println("WiFi connected!");
          Serial.print("IP address: ");
          Serial.println(WiFi.localIP());
        //************EXISTING AP SECTION ********************************************
    }

    // Start WebSocket server and assign callback
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);    

    Serial.println(F("Waiting for a Client to connect..."));
}

void processWebSocket() {

  // Look for and handle WebSocket data
  webSocket.loop();
}

// void updateDiagnostics(float ypr[3], int16_t& ac_x, int16_t& ac_y, int16_t& ac_z, float& alti, float& temp, float& pressure, float& humidity,float& voltage) {
void updateDiagnostics(float ypr[3], float& ac_x, float& ac_y, float& ac_z, float& alti, float& temp, float& pressure, float& humidity,float& voltage, byte current_state) {

    if (deviceConnected) {

        float gyro[3];
        char str[80];
        
        // gyro[_CONF.YAW_AXIS] = (ypr[_CONF.YAW_AXIS] * 180/M_PI);
        // gyro[_CONF.ROLL_AXIS] = (ypr[_CONF.ROLL_AXIS] * 180/M_PI);
        // gyro[_CONF.PITCH_AXIS] = (ypr[_CONF.PITCH_AXIS] * 180/M_PI) -90;

        gyro[_CONF.YAW_AXIS] = ypr[_CONF.YAW_AXIS];
        gyro[_CONF.ROLL_AXIS] = ypr[_CONF.ROLL_AXIS];
        gyro[_CONF.PITCH_AXIS] = ypr[_CONF.PITCH_AXIS];

        sprintf(str, "T1|%.1f|%.1f|%.1f|%.1f|%.1f|%.1f|%.2f|%.2f|%.2f|%.2f|%.2f|%d", 
                gyro[_CONF.YAW_AXIS], gyro[_CONF.PITCH_AXIS], gyro[_CONF.ROLL_AXIS], 
                ac_x, ac_y, ac_z,
                alti, temp, pressure, humidity, voltage, current_state);

        // Serial.println(str); //  DEBUG ONLY

        /* Set the value */
        // diagCharacteristic.setValue(std::string (str));  // This is a value of a single byte
        // diagCharacteristic.notify();  // Notify the client of a change
        webSocket.sendTXT(_client_num, str);

    }
}

/********
 * BLE paramCharacteristic that can be updated 
 * on a slower schedule than the main diagnistics one
 */
void updateBLEparams() {
  if (deviceConnected) {
    updatePrefs();
    updatePyros();

    //updateGuiding();
  }
}


void updatePrefs() {

    char param_str[50];

  sprintf(param_str, "T2|%d|%d|%d|%d|%d|%d", _CONF.DEBUG, _CONF.BUZZER_ENABLE, _CONF.MEMORY_CARD_ENABLED, _CONF.DATA_RECOVERY_MODE, _CONF.FORMAT_MEMORY, _CONF.SCAN_TIME_INTERVAL);
  webSocket.sendTXT(_client_num, param_str);

}

void updatePyros() {
    char str[40];
    
    sprintf(str, "T3|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d", _CONF.APOGEE_DIFF_METERS, _CONF.PARACHUTE_DELAY, _CONF.PYRO_ACTIVATION_DELAY,
            _CONF.PYRO_1_FIRE_ALTITUDE, _CONF.PYRO_2_FIRE_ALTITUDE, _CONF.PYRO_3_FIRE_ALTITUDE,_CONF.PYRO_4_FIRE_ALTITUDE,
            _CONF.AUTOMATIC_ANGLE_ABORT, _CONF.EXCESSIVE_ANGLE_THRESHOLD, _CONF.EXCESSIVE_ANGLE_TIME );

    /* Set the value */
    // pyroCharacteristic.setValue(std::string (str));  // This is a value of a single byte
    // pyroCharacteristic.notify();  // Notify the client of a change
    webSocket.sendTXT(_client_num, str);
}

void updateGuiding() {
    char str[130];
    
     if (deviceConnected) {
        sprintf(str, "T4|%d|%d|%d|%c|%c|%c|%c|%d|%d|%d|%d|%d|%d|%d|%d|%d|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f", _CONF.GUIDING_TYPE, _CONF.ROLL_CONTROL_ENABLED, _CONF.ROLL_CONTROL_TYPE,
                _CONF.SERVO_1_AXIS, _CONF.SERVO_2_AXIS, _CONF.SERVO_3_AXIS,_CONF.SERVO_4_AXIS,
                _CONF.SERVO_1_OFFSET, _CONF.SERVO_2_OFFSET, _CONF.SERVO_3_OFFSET,_CONF.SERVO_4_OFFSET,
                _CONF.SERVO_1_ORIENTATION, _CONF.SERVO_2_ORIENTATION, _CONF.SERVO_3_ORIENTATION,_CONF.SERVO_4_ORIENTATION,_CONF.MAX_FINS_TRAVEL,
                _CONF.PID_PITCH_Kp, _CONF.PID_PITCH_Ki, _CONF.PID_PITCH_Kd,
                _CONF.PID_YAW_Kp, _CONF.PID_YAW_Ki, _CONF.PID_YAW_Kd,
                _CONF.PID_ROLL_Kp, _CONF.PID_ROLL_Ki, _CONF.PID_ROLL_Kd         
                );


        /* Set the value */
        // guidingCharacteristic.setValue(std::string (str));  // This is a value of a single byte
        // guidingCharacteristic.notify();  // Notify the client of a change
        webSocket.sendTXT(_client_num, str);
        // Serial.println("Guiding Data: "); Serial.println(str);
     }
    
}