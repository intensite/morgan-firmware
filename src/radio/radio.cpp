#include "radio.h"
#include "Arduino.h"
#include "../configuration/configuration.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#define RXD2 36
#define TXD2 5
//create an RF24 object
RF24 radio(23, 26);  // CE, CSN
TinyGPSPlus gps;
//address through which two modules communicate.
const byte address[6] = "00001";
// HardwareSerial Serial2(2);


bool setupRadio() {
    radio.begin();
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Used for GPS Module
  
    //set the address
    radio.openWritingPipe(address);
    
    //Set module as transmitter
    radio.stopListening();

    // radio.printDetails();
    Serial.println("Radio configured");

    return true;
    
}


bool processRadio() {
  Serial.println("Sending message to receiver........");
  Serial.print("Task is running on: ");   Serial.println(xPortGetCoreID());
  //Send message to receiver
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));

  return true;
};


// Used to broadcast GPS coordinate on the radio signal
void updateRadioGPS( ){


   char output_buffer[100] = "";
   char tmp_buffer[25] = "";
   
   while(Serial2.available()){           //While there are characters to come from the GPS
     gps.encode(Serial2.read());         //This feeds the serial NMEA data into the library one char at a time
   }
 
   if(gps.location.isUpdated()){         //This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in

        //Get the latest info from the gps object which it derived from the data sent by the GPS unit
       Serial.println("Satellite Count:");
       Serial.println(gps.satellites.value());
       Serial.println("Latitude:");
       Serial.println(gps.location.lat(), 6);
       Serial.println("Longitude:");
       Serial.println(gps.location.lng(), 6);
       Serial.println("Speed M/S:");
       Serial.println(gps.speed.mps());
       Serial.println("Altitude Meters:");
       Serial.println(gps.altitude.meters());
       Serial.println("Time: ");
       Serial.println(gps.time.value());
       Serial.println("");
   
       
    //    strcpy(output_buffer,"Satellite Count: ");
    //    itoa(gps.satellites.value(), tmp_buffer, 10); 
    //    strcat(output_buffer, tmp_buffer);
    //    radio.write(&output_buffer, sizeof(output_buffer));
       
    //    strcpy(output_buffer,"Latitude: ");
    //    dtostrf(gps.location.lat(), 10, 6, tmp_buffer); //10 is mininum width, 6 is precision
    //    strcat(output_buffer, tmp_buffer);  
    //    radio.write(&output_buffer, sizeof(output_buffer));
       
    //    strcpy(output_buffer,"Longitude: ");
    //    dtostrf(gps.location.lng(), 10, 6, tmp_buffer); //10 is mininum width, 6 is precision
    //    strcat(output_buffer, tmp_buffer);  
    //    radio.write(&output_buffer, sizeof(output_buffer));        
   
    //    strcpy(output_buffer,"Speed M/S: ");
    //    dtostrf(gps.speed.mps(), 6, 2, tmp_buffer); //6 is mininum width, 2 is precision
    //    strcat(output_buffer, tmp_buffer);  
    //    radio.write(&output_buffer, sizeof(output_buffer));      
   
    //    strcpy(output_buffer,"Altitude Meters: ");
    //    dtostrf(gps.altitude.meters(), 6, 2, tmp_buffer); //6 is mininum width, 2 is precision
    //    strcat(output_buffer, tmp_buffer);  
    //    radio.write(&output_buffer, sizeof(output_buffer));      
   
    //    strcpy(output_buffer,"Time: ");
    //    ltoa(gps.time.value(), tmp_buffer, 10); 
    //    strcat(output_buffer, tmp_buffer);  
    //    radio.write(&output_buffer, sizeof(output_buffer));   
   
    //    Serial.println("Data sent to receiver....");
       
   } 

}
