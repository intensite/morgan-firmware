#include <Arduino.h>

// void buzz(unsigned char targetPin, uint16_t frequency, int8_t length) {
void buzz(uint8_t targetPin, int32_t frequency, int32_t length)  {
  uint16_t delayValue = 0;
  if(frequency>0) {
    delayValue = (1000000/frequency)/2; // calculate the delay value between transitions
  }
  // delayValue = delayValue / 2;
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  uint16_t numCycles = frequency * length/ 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to 
  //// get the total number of cycles to produce
  for (uint16_t i=0; i < numCycles; i++){ // for the calculated length of time...
    digitalWrite(targetPin,HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin,LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }

}