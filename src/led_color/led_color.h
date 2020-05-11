#include "../config.h"
#include "Arduino.h"

#ifndef LED_COLOR_FILE_H
#define LED_COLOR_FILE_H

#define LED_COLOR_OFF 0
#define LED_COLOR_RED 4
#define LED_COLOR_GREEN 2
#define LED_COLOR_BLUE 1
#define LED_COLOR_YELLOW 6
#define LED_COLOR_CYAN 3
#define LED_COLOR_PURPLE 5
#define LED_COLOR_WHITE 7 

void led_color(byte color_code);
void led_color_rgb(int32_t color_code);

#endif // LED_COLOR_FILE_H
