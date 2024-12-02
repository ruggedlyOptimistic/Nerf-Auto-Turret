#include <PS4Controller.h>

// All of this needs to be changed so that the header file actually looks and functions like a header file
// This file is currently structured more like a .cpp file

class PS4_CONTROLLER
{
    private:
        const char* mac;
        bool paired;

        int8_t rx;
        int8_t rxOffset;
        int8_t ly;
        int8_t lyOffset;
        int8_t rt;

        bool square = false;
        bool triangle = false;
        bool circle = false;
        bool cross = false;
        bool r1 = false;
        bool options = false;

    public:
        PS4_CONTROLLER(const char* mac_address);
        
        // Methods
        // void begin(const char* mac);
        // void attach()
        
        
        void toggleAutoTargetMode();
        void notify();
        void onConnect();
        void onDisconnect();
        bool squareDown();
        bool triangleDown();
        bool circleDown();
        bool crossDown();
        bool r1Down();
        bool optionsDown();
        int8_t rxValue();
        int8_t lyValue();
};
