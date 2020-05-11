#include "led_color.h"

void led_color(byte color_code) {

    /***
     LED_COLOR_OFF 0
     LED_COLOR_RED 4
     LED_COLOR_GREEN 2
     LED_COLOR_BLUE 1
     LED_COLOR_YELLOW 6
     LED_COLOR_CYAN 3
     LED_COLOR_PURPLE 5
     LED_COLOR_WHITE 7 
     * 
     */

    // Used the bits of the color code to turn on or off different color leds (Total 8 colors)
    (color_code &  0x1 )  ? digitalWrite(B_LED, LOW) : digitalWrite(B_LED, HIGH);
    (color_code &  0x2 )  ? digitalWrite(G_LED, LOW) : digitalWrite(G_LED, HIGH);
    (color_code &  0x4 )  ? digitalWrite(R_LED, LOW) : digitalWrite(R_LED, HIGH);
}

void led_color_rgb(int32_t color_code) {

    #define R_LED_CHANNEL 5
    #define G_LED_CHANNEL 6
    #define B_LED_CHANNEL 7

    // Common Cathode 
    // byte r = ((color_code >> 16) & 0xFF) / 255.0;  // Extract the RR byte
    // byte g = ((color_code >> 8) & 0xFF) / 255.0;   // Extract the GG byte
    // byte b = ((color_code) & 0xFF) / 255.0;        // Extract the BB byte

    // Common Anode
    byte r = ((color_code >> 16) & 0xFF) ;  // Extract the RR byte
    byte g = ((color_code >> 8) & 0xFF) ;   // Extract the GG byte
    byte b = ((color_code) & 0xFF) ;        // Extract the BB byte

    // For common Anode RGB, substract the value from 255  LOW=ON HIGH=OFF
    ledcWrite(R_LED_CHANNEL, 255-r);
    ledcWrite(G_LED_CHANNEL, 255-g);
    ledcWrite(B_LED_CHANNEL, 255-b);



}