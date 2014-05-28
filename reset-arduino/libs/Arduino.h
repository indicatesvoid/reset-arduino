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
#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
    #include <termios.h>
    #include <sys/ioctl.h>
    #include <getopt.h>
    #include <dirent.h>
#else
    #include <windows.h>
    #include <tchar.h>
    #include <iostream>
    #include <string.h>
    #include <setupapi.h>
    #include <regstr.h>
    #define MAX_SERIAL_PORTS 256
     #include <winioctl.h>
    #ifdef __MINGW32__
            #define INITGUID
            #include <initguid.h> // needed for dev-c++ & DEFINE_GUID
    #endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>


class Arduino {
    public:
        Arduino();
		~Arduino();
    
        bool connect(int baud);
		void close();
        void reset();
    
    private:

    // METHODS

        std::vector<std::string> findArduinos();
        std::vector<std::string> buildDeviceList();
        void listDetectedDevices();
        std::string chooseActiveDevice();

    // VARIABLES

	#ifdef TARGET_WIN32

		char 		** portNamesShort;//[MAX_SERIAL_PORTS];
		char 		** portNamesFriendly; ///[MAX_SERIAL_PORTS];
		HANDLE  	hComm;		// the handle to the serial port pc
		int	 		nPorts;
		bool 		bPortsEnumerated;
		void 		enumerateWin32Ports();
		COMMTIMEOUTS 	oldTimeout;	// we alter this, so keep a record

	#else
		int fd;
	#endif

        std::string device_path;  
        std::vector<std::string> devices;
        std::vector<std::string> arduinos;
		bool bInited;
    
        struct Device {
            int index;
            std::string path;
        };
};


