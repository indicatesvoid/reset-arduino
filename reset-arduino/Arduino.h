//
//  Arduino.h
//  Trenchcoat
//
//  Created by William A. Clark on 6/14/13.
//  Do as ye please.
//

#include <vector>
#include <string>
#include <map>

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <dirent.h>

typedef enum {
    LEONARDO,
    OTHER
} Boards;

class Arduino {
    public:
        Arduino();
    
        void find();
        void setBaud(int baud = 9600);
        bool openPort();
        void closePort();
        bool connect(int baud = 9600, bool verbose = true);
        void reset(int _board = OTHER);
    
    private:

    // METHODS

        std::vector<std::string> findArduinos();
        std::vector<std::string> buildDeviceList();
        void listDetectedDevices();
        std::string chooseActiveDevice();

    // VARIABLES
        bool bIsInited, bArduinoFound;
    
        struct termios oldoptions;
        unsigned long currentBaud;

        std::string device_path;
        int fd;
        std::vector<std::string> devices;
        std::vector<std::string> arduinos;
    
        struct Device {
            int index;
            std::string path;
        };
};


