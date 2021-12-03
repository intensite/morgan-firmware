#ifndef __simulation_h__
#define __simulation_h__


class Simulation {
    private:

    public:
        static char* message;

        Simulation();
        void handleReceivedMessage(char* msg);
 
};

#endif
