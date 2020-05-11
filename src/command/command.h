#ifndef __command_h__
#define __command_h__

#include <SimpleCLI.h>

class CliCommand {
    private:
        SimpleCLI cli;
        Command cmdGet;
        Command cmdSet;

    public:
        CliCommand();
        void handleReceivedMessage(const char* msg);
        void handleSerial();
        void processGetCommand(const char*);
        void processSetCommand(const char* setting, const char* value);
};

#endif