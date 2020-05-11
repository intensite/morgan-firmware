#pragma once
#include "Arduino.h"
#include "../lib/Servo.h"
#include "../config.h"

void moveServo(float _ypr[]);
void moveServoTVC(float _ypr[]);
void moveServoFins(float _ypr[]);
void setupServo();
void testServo();
void disableServo();