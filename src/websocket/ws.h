#pragma once

#include "../command/command.h"
#include "../storage/LogSystem.h"
#include <WebSocketsServer.h>



void setupWebSocket(CliCommand& cliPtr);
void processWebSocket();
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length);


// void updateDiagnostics(float ypr[3], int16_t ac_x, int16_t ac_y, int16_t ac_z);
// void updateDiagnostics(float ypr[3], int16_t& ac_x, int16_t& ac_y, int16_t& ac_z, float& alti, float& temp, float& pressure, float& humidity, float& voltage);
void updateDiagnostics(float ypr[3], float& ac_x, float& ac_y, float& ac_z, float& alti, float& temp, float& pressure, float& humidity, float& voltage, byte current_state);
void updateBLEparams();
void updatePrefs();
void updatePyros();
void updateGuiding();
// void uploadFlightData(lr::LogRecord logRecord);
