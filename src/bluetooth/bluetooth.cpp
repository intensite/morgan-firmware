// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>
// #include "Arduino.h"
// #include "../configuration/configuration.h"
// #include "../command/command.h"
// #include "./bluetooth.h"
// #include "../lib/PString.h"
// #include "../lib/Streaming.h"

// //#include "helper_3dmath.h"

// /* Define the UUID for our Custom Service */
// #define serviceID BLEUUID((uint16_t)0x1700)

// // Input Commands Caracteristic (W/O)  Command input
// BLECharacteristic commandCharacteristic(
//   BLEUUID((uint16_t)0x1A01), 
//   // BLECharacteristic::PROPERTY_READ | 
//   // BLECharacteristic::PROPERTY_WRITE | 
//   BLECharacteristic::PROPERTY_WRITE |
//   BLECharacteristic::PROPERTY_NOTIFY
// );
// // Diagnostics Caracteristic (R/O)  Page Diagnostiques
// BLECharacteristic diagCharacteristic(
//   BLEUUID((uint16_t)0x1A00), 
//   BLECharacteristic::PROPERTY_READ | 
//   BLECharacteristic::PROPERTY_NOTIFY
// );
// // Parameters Caracteristic (R/O)  Page preferences
// BLECharacteristic paramCharacteristic(
//   BLEUUID((uint16_t)0x1A02), 
//   BLECharacteristic::PROPERTY_READ | 
//   BLECharacteristic::PROPERTY_NOTIFY
// );
// // Accelerometer Caracteristic (R/O)
// BLECharacteristic pyroCharacteristic(
//   BLEUUID((uint16_t)0x1A03), 
//   BLECharacteristic::PROPERTY_READ | 
//   BLECharacteristic::PROPERTY_NOTIFY
// );
// // // Guiding Caracteristic (R/O)
// BLECharacteristic guidingCharacteristic(
//   BLEUUID((uint16_t)0x1A04), 
//   BLECharacteristic::PROPERTY_READ | 
//   BLECharacteristic::PROPERTY_NOTIFY
// );
// // // // Flight Data Download Caracteristic (R/O)
// BLECharacteristic flightDataCharacteristic(
//   BLEUUID((uint16_t)0x1A05), 
//   BLECharacteristic::PROPERTY_READ | 
//   BLECharacteristic::PROPERTY_NOTIFY
// );

// /* Define the UUID for our Custom Service */
// #define serviceID BLEUUID((uint16_t)0x1700)
// CliCommand clii; // Passed from the setupBLE function to process the received commands

// void processCommand(const char* msg) {
//   // if (msg) {
//     // Serial.println("Inside processCommand");
//     clii.handleReceivedMessage(msg);
//   // } else {
//   //   Serial.println("No message/command received from bluetooth!");
//   // }
// }

// /* This function handles the server callbacks */
// bool deviceConnected = false;
// class ServerCallbacks: public BLEServerCallbacks {
//     void onConnect(BLEServer* MyServer) {
//       deviceConnected = true;
//     };

//     void onDisconnect(BLEServer* MyServer) {
//       deviceConnected = false;
//     }
// };

// class CharacteristicCallbacks: public BLECharacteristicCallbacks {

//       // Define a callback type: a pointer to a function taking no
//       // arguments and returning void.
//       typedef void (*callback_t)(const char*);

//     public:
//       CharacteristicCallbacks(callback_t _callback)         
//         // Initialize internal callback with the one given as parameter.
//         : callback(_callback)
//         // empty function body
//         {}

//      void onWrite(BLECharacteristic *characteristic) {
//           //return the pointer to the register containing the current value of the characteristic
//           std::string rxValue = characteristic->getValue(); 
//               //  Serial.print("Value to be passed to callback: ");
//               //  Serial.println(rxValue.c_str());
          
//           //check if there are data (size greater than zero)
//           if (rxValue.length() > 0) {
 
//               // for (int i = 0; i < rxValue.length(); i++) {
//               //   Serial.print(rxValue[i]);
//               //  }

//                Serial.print("Calling CLI.processSetCommand() with : "); Serial.println(rxValue.c_str());
//                callback(rxValue.c_str());
//           }
//      }//onWrite

//      private:
//     // The callback is kept as private internal data.
//     callback_t callback;
// };


// void setupBLE(CliCommand& cliPtr) {

//   clii = cliPtr; // Get a reference to the main's CLI object to parse the commands received.

//    // Create and name the BLE Device
//   BLEDevice::init("MORGAN flight computer");
//   BLEDevice::setMTU(130);

//   /* Create the BLE Server */
//   BLEServer *MyServer = BLEDevice::createServer();
//   MyServer->setCallbacks(new ServerCallbacks());  // Set the function that handles Server Callbacks

//   /* Add a service to our server */
//   // Note: The second parameter is the numHandles The maximum number of handles associated with this service. 
//   // The defaut 15 doesn't allow for more that 3 characteristics. 30 seems to work fine.
//   BLEService *customService = MyServer->createService(BLEUUID((uint16_t)0x1700), 30 , 0);  

//   /* Add a characteristic to the service */
//   customService->addCharacteristic(&diagCharacteristic);  //diagCharacteristic was defined above
//   customService->addCharacteristic(&commandCharacteristic);  
//   customService->addCharacteristic(&paramCharacteristic);  
//   customService->addCharacteristic(&pyroCharacteristic);  
//   customService->addCharacteristic(&guidingCharacteristic);  
//   customService->addCharacteristic(&flightDataCharacteristic);  

//   /* Add Descriptors to the Characteristic*/
//   // commandCharacteristic.addDescriptor(new BLE2902());  //Add this line only if the characteristic has the Notify property
//   diagCharacteristic.addDescriptor(new BLE2902());  //Add this line only if the characteristic has the Notify property
//   paramCharacteristic.addDescriptor(new BLE2902());  //Add this line only if the characteristic has the Notify property
//   pyroCharacteristic.addDescriptor(new BLE2902());  //Add this line only if the characteristic has the Notify property
//   guidingCharacteristic.addDescriptor(new BLE2902());  //Add this line only if the characteristic has the Notify property
//   flightDataCharacteristic.addDescriptor(new BLE2902());  //Add this line only if the characteristic has the Notify property

//   // Callback use to receive commands
//   commandCharacteristic.setCallbacks(new CharacteristicCallbacks(processCommand));

//   BLEDescriptor VariableDescriptor(BLEUUID((uint16_t)0x2901));    /*```````````````````````````````````````````````````````````````*/
//   VariableDescriptor.setValue("gyro pitch, roll, yaw");           /* Use this format to add a hint for the user. This is optional. */
//   diagCharacteristic.addDescriptor(&VariableDescriptor);          /*```````````````````````````````````````````````````````````````*/

//   /* Configure Advertising with the Services to be advertised */
//   MyServer->getAdvertising()->addServiceUUID(serviceID);

//   // Start the service
//   customService->start();

//   // Start the Server/Advertising
//   MyServer->getAdvertising()->start();

//   Serial.println(F("Waiting for a Client to connect..."));
// }

// void updateDiagnostics(float ypr[3], int16_t& ac_x, int16_t& ac_y, int16_t& ac_z, float& alti, float& temp, float& pressure, float& humidity,float& voltage) {

//     if (deviceConnected) {

//     float gyro[3];
//     char str[80];
    
//     // gyro[_CONF.YAW_AXIS] = (ypr[_CONF.YAW_AXIS] * 180/M_PI);
//     // gyro[_CONF.ROLL_AXIS] = (ypr[_CONF.ROLL_AXIS] * 180/M_PI);
//     // gyro[_CONF.PITCH_AXIS] = (ypr[_CONF.PITCH_AXIS] * 180/M_PI) -90;

//     gyro[_CONF.YAW_AXIS] = ypr[_CONF.YAW_AXIS];
//     gyro[_CONF.ROLL_AXIS] = ypr[_CONF.ROLL_AXIS];
//     gyro[_CONF.PITCH_AXIS] = ypr[_CONF.PITCH_AXIS];

//     sprintf(str, "%.1f|%.1f|%.1f|%d|%d|%d|%.2f|%.2f|%.2f|%.2f|%.2f", 
//               gyro[_CONF.YAW_AXIS], gyro[_CONF.PITCH_AXIS], gyro[_CONF.ROLL_AXIS], 
//               ac_x, ac_y, ac_z,
//               alti, temp, pressure, humidity, voltage);

//     /* Set the value */
//     diagCharacteristic.setValue(std::string (str));  // This is a value of a single byte
//     diagCharacteristic.notify();  // Notify the client of a change

//       // updateOrientation(ypr);
//       // updateBLEparams();
//       // updateAccels(ac_x, ac_y, ac_z);
//       //updateEnvironment();

//     }
// }

// /********
//  * BLE paramCharacteristic that can be updated 
//  * on a slower schedule than the main diagnistics one
//  */
// void updateBLEparams() {
//   if (deviceConnected) {
//     updatePrefs();
//     updatePyros();

//     //updateGuiding();
//   }
// }


// void updatePrefs() {

//     char param_str[50];

//   sprintf(param_str, "%d|%d|%d|%d|%d|%d", _CONF.DEBUG, _CONF.BUZZER_ENABLE, _CONF.MEMORY_CARD_ENABLED, _CONF.DATA_RECOVERY_MODE, _CONF.FORMAT_MEMORY, _CONF.SCAN_TIME_INTERVAL);
//   paramCharacteristic.setValue(std::string (param_str));
//   paramCharacteristic.notify();  // Notify the client of a change

// }

// void updatePyros() {
//     char str[40];
    
//     sprintf(str, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%d", _CONF.APOGEE_DIFF_METERS, _CONF.PARACHUTE_DELAY, _CONF.PYRO_ACTIVATION_DELAY,
//             _CONF.PYRO_1_FIRE_ALTITUDE, _CONF.PYRO_2_FIRE_ALTITUDE, _CONF.PYRO_3_FIRE_ALTITUDE,_CONF.PYRO_4_FIRE_ALTITUDE,
//             _CONF.AUTOMATIC_ANGLE_ABORT, _CONF.EXCESSIVE_ANGLE_THRESHOLD, _CONF.EXCESSIVE_ANGLE_TIME );

//     /* Set the value */
//     pyroCharacteristic.setValue(std::string (str));  // This is a value of a single byte
//     pyroCharacteristic.notify();  // Notify the client of a change
// }
// void updateGuiding() {
//     char str[130];
    
//      if (deviceConnected) {
//         sprintf(str, "%d|%d|%d|%c|%c|%c|%c|%d|%d|%d|%d|%d|%d|%d|%d|%d|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f", _CONF.GUIDING_TYPE, _CONF.ROLL_CONTROL_ENABLED, _CONF.ROLL_CONTROL_TYPE,
//                 _CONF.SERVO_1_AXIS, _CONF.SERVO_2_AXIS, _CONF.SERVO_3_AXIS,_CONF.SERVO_4_AXIS,
//                 _CONF.SERVO_1_OFFSET, _CONF.SERVO_2_OFFSET, _CONF.SERVO_3_OFFSET,_CONF.SERVO_4_OFFSET,
//                 _CONF.SERVO_1_ORIENTATION, _CONF.SERVO_2_ORIENTATION, _CONF.SERVO_3_ORIENTATION,_CONF.SERVO_4_ORIENTATION,_CONF.MAX_FINS_TRAVEL,
//                 _CONF.PID_PITCH_Kp, _CONF.PID_PITCH_Ki, _CONF.PID_PITCH_Kd,
//                 _CONF.PID_YAW_Kp, _CONF.PID_YAW_Ki, _CONF.PID_YAW_Kd,
//                 _CONF.PID_ROLL_Kp, _CONF.PID_ROLL_Ki, _CONF.PID_ROLL_Kd         
//                 );


//         /* Set the value */
//         guidingCharacteristic.setValue(std::string (str));  // This is a value of a single byte
//         guidingCharacteristic.notify();  // Notify the client of a change

//         // Serial.println("Guiding Data: "); Serial.println(str);
//      }
    
// }

// void uploadFlightData(lr::LogRecord logRecord) {

//     /* This function knows the array will be of length RECORD_SIZE */
//     char buffer[100];
//     PString str(buffer, sizeof(buffer));

//     str << logRecord._timestamp << ",";
//     str << logRecord._altitude << ",";
//     str << logRecord._pitch << ",";
//     str << logRecord._roll << ",";
//     str << logRecord._pitchServo << ",";
//     str << logRecord._rollServo << ",";
//     str << logRecord._parachute << ",";
//     str << logRecord._abort << ",";
//     str << logRecord._temperature << ",";
//     str << logRecord._battery << ",";
//     str << logRecord._gForces; // << endl;

//   // Serial.print(buffer);

//      if (deviceConnected) {
//         /* Set the value */
//         flightDataCharacteristic.setValue(std::string (buffer));  // This is a value of a single byte
//         flightDataCharacteristic.notify();  // Notify the client of a change
//      }

// }


// // void BLESendChunks(std::string str)
// // {
// // 	std::string substr;
// // 	if (deviceConnected) {
// // 		for (int k = 0; k < str.length(); k += _min(str.length(), 20)) {
// // 			substr = str.substr(k, _min(str.length() - k, 20));
// // 			pCharacteristic->setValue(substr);
// // 			pCharacteristic->notify();
// // 		}
// // 	}
// // }



