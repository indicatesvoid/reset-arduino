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

class Arduino {
    public:
        Arduino();
    
        bool connect(int baud);
        void reset();
    
    private:

    // METHODS

        std::vector<std::string> findArduinos();
        std::vector<std::string> buildDeviceList();
        void listDetectedDevices();
        std::string chooseActiveDevice();

    // VARIABLES

        std::string device_path;
        int fd;
        std::vector<std::string> devices;
        std::vector<std::string> arduinos;
    
        struct Device {
            int index;
            std::string path;
        };
};


