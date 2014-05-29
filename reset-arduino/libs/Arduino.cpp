//
//  Arduino.cpp
//  reset-arduino
//
//  Created by William A. Clark on 6/14/13.
//  Based off of the OpenFrameworks ofSerial class
//

#include "Arduino.h"

using namespace std;

//////////////////////////////////////////////
//                                          //
//              PUBLIC METHODS              //
//                                          //
//////////////////////////////////////////////

Arduino::Arduino() {
	devices = buildDeviceList();
	arduinos = findArduinos();
}

Arduino::~Arduino() {
}

void Arduino::find() {
    bArduinoFound = false;
    
    if(arduinos.size() == 0) {
        std::cout << "No Arduinos found. Please plug one in and run the application again.\n" << endl;
        return;
    }
    
    if(arduinos.size() > 1) device_path = chooseActiveDevice();
    else device_path = arduinos[0];
    bArduinoFound = true;
}

void Arduino::setBaud(int baud) {
    // set baud rate - currently OSX and Linux only, Windows support forthcoming

    // safety until Windows support comes
    #ifdef TARGET_WIN32
        return;
    #else
    
    struct termios ComParams;
    tcgetattr(fd, &ComParams);
    oldoptions = ComParams;
    
    switch(baud){
        case 300: 	cfsetispeed(&ComParams,B300);
            cfsetospeed(&ComParams,B300);
            break;
        case 1200:
            cfsetispeed(&ComParams,B1200);
            cfsetospeed(&ComParams,B1200);
            break;
        case 2400: 	cfsetispeed(&ComParams,B2400);
            cfsetospeed(&ComParams,B2400);
            break;
        case 4800: 	cfsetispeed(&ComParams,B4800);
            cfsetospeed(&ComParams,B4800);
            break;
        case 9600: 	cfsetispeed(&ComParams,B9600);
            cfsetospeed(&ComParams,B9600);
            break;
        case 14400: 	cfsetispeed(&ComParams,B14400);
            cfsetospeed(&ComParams,B14400);
            break;
        case 19200: 	cfsetispeed(&ComParams,B19200);
            cfsetospeed(&ComParams,B19200);
            break;
        case 28800: 	cfsetispeed(&ComParams,B28800);
            cfsetospeed(&ComParams,B28800);
            break;
        case 38400: 	cfsetispeed(&ComParams,B38400);
            cfsetospeed(&ComParams,B38400);
            break;
        case 57600:  cfsetispeed(&ComParams,B57600);
            cfsetospeed(&ComParams,B57600);
            break;
        case 115200: cfsetispeed(&ComParams,B115200);
            cfsetospeed(&ComParams,B115200);
            break;
            
        default:
            cfsetispeed(&ComParams,B9600);
            cfsetospeed(&ComParams,B9600);
            std::cout << "\rCannot set " << baud << " bps, setting to 9600" << std::endl;
            break;
    }
    
    ComParams.c_cflag |= (CLOCAL | CREAD);
    ComParams.c_cflag &= ~PARENB;
    ComParams.c_cflag &= ~CSTOPB;
    ComParams.c_cflag &= ~CSIZE;
    ComParams.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
    ComParams.c_oflag &= (tcflag_t) ~(OPOST);
    ComParams.c_cflag |= CS8;
    tcsetattr( fd, TCSANOW, &ComParams );
    
    unsigned long retrievedBaud = cfgetospeed(&ComParams);
    printToConsole("Setting baud rate " + to_string(retrievedBaud));
    currentBaud = retrievedBaud;
    
#endif
}

bool Arduino::openPort() {
    bIsInited = false;
    
    //---------------------------------------------
#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
	//---------------------------------------------
    
    // open the serial device
    fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
    
    if(fd == -1){
        std::cout << "\rUnable to open port" << std::endl;
        return false;
    }
    
    // safety - set baud rate once more after open
    setBaud((int)currentBaud);
    
    std::cout << "\rSuccess in opening serial port at " << device_path << std::endl;
    struct termios opts;
    tcgetattr(fd, &opts);
    unsigned long retrievedBaud = cfgetospeed(&opts);
    
    std::cout << "\rAt baud rate " << to_string(retrievedBaud) << std::endl;
    
    bIsInited = true;
    return true;
#endif
    
    //---------------------------------------------
#ifdef TARGET_WIN32
	//---------------------------------------------
    hComm = CreateFileA(device_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    
    if (hComm == INVALID_HANDLE_VALUE) {
        std::cout << "\rUnable to open port" << std::endl;
        return false;
    }
    
    COMMCONFIG cfg;
    DWORD cfgSize;
    char  buf[80];
    cfgSize = sizeof(cfg);
    // arduino is standard 8N1. We'll use 9600 baud since we're not doing anything intensive
    sprintf(buf, "baud=9600 parity=N data=8 stop=1");
    
    GetCommConfig(hComm, &cfg, &cfgSize);
    
#if (_MSC_VER)       // microsoft visual studio
    // msvc doesn't like BuildCommDCB,
    //so we need to use this version: BuildCommDCBA
    if (!BuildCommDCBA(buf, &cfg.dcb)){
        std::cout << "\rERROR: Something went wrong with the serial connection. Insert unhelpful error message here." << std::endl;
    }
#else
    if (!BuildCommDCB(buf, &cfg.dcb)){
        std::cout << "\rSomething went wrong with the serial connection. Insert unhelpful error message here." << std::endl;
    }
#endif
    
    if (!SetCommState(hComm, &cfg.dcb)){
        std::cout << "\rERROR: Can't set comm state" << std::endl;
    }
    
    // Set communication timeouts (NT)
    COMMTIMEOUTS tOut;
    GetCommTimeouts(hComm, &oldTimeout);
    tOut = oldTimeout;
    // Make timeout so that:
    // - return immediately with buffered characters
    tOut.ReadIntervalTimeout = MAXDWORD;
    tOut.ReadTotalTimeoutMultiplier = 0;
    tOut.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts(hComm, &tOut);
    
    std::cout << "\rSuccess in opening serial port" << std::endl;
    bInited = true;
    return true;
	//---------------------------------------------
#endif
	//---------------------------------------------
    
    return false;
}

bool Arduino::connect(int baud, bool verbose) {
	find();
    setBaud(baud);
    bool success = openPort();
    
    return success;
}

void Arduino::closePort(){

	//---------------------------------------------
#ifdef TARGET_WIN32
	//---------------------------------------------
	if (bIsInited){
		SetCommTimeouts(hComm, &oldTimeout);
		CloseHandle(hComm);
		hComm = INVALID_HANDLE_VALUE;
		bInited = false;
	}
	//---------------------------------------------
#else
	//---------------------------------------------
	if (bIsInited){
		tcsetattr(fd, TCSANOW, &oldoptions);
		::close(fd);
	}
	// [CHECK] -- anything else need to be reset?
	//---------------------------------------------
#endif
	//---------------------------------------------

}

void Arduino::reset(int _board) {
    if(!bArduinoFound) find();
    sleep(1);
    
    if(_board == LEONARDO) {
        // force reset using 1200bps open/close
        // not working?
        setBaud(57600);
        closePort();
        openPort();
        closePort();
        sleep(1);
        setBaud(1200);
        openPort();
        closePort();
        openPort();
        closePort();
        sleep(1);
    }
    
    else {
        // set a high bit over the serial line, wait, then reset
        // this will smack the Arduino into resetting
        std::cout<<"\rResetting Arduino..." << std::endl;
        
        setBaud(9600);
        openPort();
        
        #ifdef TARGET_WIN32
            // with thanks to http://stackoverflow.com/questions/18539104/controlling-dtr-and-rts-pin-of-serial-port-in-c-on-windows-platform
            EscapeCommFunction(hComm,SETDTR);
            Sleep(500);
            EscapeCommFunction(hComm,CLRDTR);
        #else
            ioctl(fd, TIOCMBIS, TIOCM_DTR);
            usleep(500);
            ioctl(fd, TIOCMBIC, TIOCM_DTR);
        #endif
    }
}

//////////////////////////////////////////////
//                                          //
//              PRIVATE METHODS             //
//                                          //
//////////////////////////////////////////////

inline void Arduino::printToConsole(std::string str) {
    std::cout << "\r" << str << std::endl;
}

void Arduino::listDetectedDevices() {
    for(int i = 0; i < arduinos.size(); i++) {
        string index = to_string(i);
        cout << "[" << index << "] " << arduinos[i] << endl;
    }
}

// TO-DO :: WINDOWS SUPPORT
vector<string> Arduino::findArduinos() {
    // modified from code by Stephen Braitsch
    // https://github.com/braitsch
    
    // detect available serial ports //
    std::vector<std::string> _arduinos;
    string arduinoPort = "";
    for (int i=0; i < devices.size(); i++) {
        string n = devices[i];
        if (std::string::npos != n.find("/dev/tty.usbserial")) _arduinos.push_back(n);      // for boards that use FTDI serial
        if(std::string::npos != n.find("/dev/tty.usbmodem")) _arduinos.push_back(n);        // for newer boards (Mega, Uno, etc)
    }
    
    return _arduinos;
}

string Arduino::chooseActiveDevice() {
    cout << "\rMultiple Arduinos detected:/n";
    listDetectedDevices();
    cout << "\rPlease choose active device: ";
    string input;
    int deviceIndex;
    bool input_is_int = false;
    bool input_is_valid = false;
    cin >> input;
    
    // test if input is a valid integer
    std::string::const_iterator it = input.begin();
    while (it != input.end() && isdigit(*it)) ++it;
    if(!input.empty() && it == input.end()) input_is_int = true;
    
    if(input_is_int) {
        // user passed device index as input
        deviceIndex = stoi(input);
        // test if user input is valid
        if(deviceIndex < arduinos.size() + 1) input_is_valid = true;
    }
    
    else {
        // test if user input is valid path
        for(int i = 0; i < arduinos.size(); i++) {
            if(input == arduinos[i]) {
                input_is_valid = true;
                break;
            }
        }
    }
    
    if(input_is_valid) {
        if(input_is_int) return arduinos[deviceIndex];
        else return input;
    }
    
    else {
        cout << "\rSelection invalid. Please make sure input is a valid port index or path." << std::endl;
        chooseActiveDevice();
    }
    
    return arduinos[0]; // default/fallback, just in case.
}

vector<string> Arduino::buildDeviceList() {
    // mostly nabbed from ofSerial class
    vector <string> prefixMatch;
    prefixMatch.push_back("cu.");
    prefixMatch.push_back("tty.");
    
    vector<string> _devices;
    
	//---------------------------------------------
	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
	//---------------------------------------------
		DIR *dir;
		struct dirent *entry;
		dir = opendir("/dev");
    
		string deviceName	= "";
    
        if (dir == NULL) std::cout << "\rError listing devices in /dev" << std::endl;
		
		else {
			//for each device
			while((entry = readdir(dir)) != NULL){
				deviceName = (char *)entry->d_name;
            
				//we go through the prefixes
				for(int k = 0; k < (int)prefixMatch.size(); k++){
					//if the device name is longer than the prefix
					if( deviceName.size() > prefixMatch[k].size() ){
						//do they match ?
						if( deviceName.substr(0, prefixMatch[k].size()) == prefixMatch[k].c_str() ){
							_devices.push_back("/dev/"+deviceName);
							break;
						}
					}
				}
			}
			closedir(dir);
		}
    
		return _devices;
		//---------------------------------------------
		#endif
		//---------------------------------------------
		//---------------------------------------------
		#if defined( TARGET_WIN32 )
		//---------------------------------------------
			enumerateWin32Ports();
			for (int i = 0; i < nPorts; i++) {
				_devices.push_back(portNamesFriendly[i]);
			}

			return _devices;
		//---------------------------------------------
		#endif
		//---------------------------------------------
}

//---------------------------------------------
#ifdef TARGET_WIN32
//---------------------------------------------

//------------------------------------
// needed for serial bus enumeration:
//4d36e978-e325-11ce-bfc1-08002be10318}
DEFINE_GUID(GUID_SERENUM_BUS_ENUMERATOR, 0x4D36E978, 0xE325,
	0x11CE, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18);
//------------------------------------

void Arduino::enumerateWin32Ports(){

	// copy and pasted almost completely from
	// https://github.com/lian/ofx-dev/blob/master/libs/openFrameworks/communication/ofSerial.cpp
	// I claim no credit
	// -- WC

	if (bPortsEnumerated == true) return;

	HDEVINFO hDevInfo = NULL;
	SP_DEVINFO_DATA DeviceInterfaceData;
	int i = 0;
	DWORD dataType, actualSize = 0;
	unsigned char dataBuf[MAX_PATH + 1];

	// Reset Port List
	nPorts = 0;
	// Search device set
	hDevInfo = SetupDiGetClassDevs((struct _GUID *)&GUID_SERENUM_BUS_ENUMERATOR, 0, 0, DIGCF_PRESENT);
	if (hDevInfo){
		while (TRUE){
			ZeroMemory(&DeviceInterfaceData, sizeof(DeviceInterfaceData));
			DeviceInterfaceData.cbSize = sizeof(DeviceInterfaceData);
			if (!SetupDiEnumDeviceInfo(hDevInfo, i, &DeviceInterfaceData)){
				// SetupDiEnumDeviceInfo failed
				break;
			}

			if (SetupDiGetDeviceRegistryProperty(hDevInfo,
				&DeviceInterfaceData,
				SPDRP_FRIENDLYNAME,
				&dataType,
				dataBuf,
				sizeof(dataBuf),
				&actualSize)){

				//sprintf(portNamesFriendly[nPorts], "%s", dataBuf);
				//portNamesShort[nPorts][0] = 0;

				// turn blahblahblah(COM4) into COM4

				char *   begin = NULL;
				char *   end = NULL;
				begin = strstr((char *)dataBuf, "COM");

				std::cout << dataBuf;
				
				if (begin)
				{
					//end = strstr(begin, ")");
					//if (end)
					//{
					//	*end = 0;   // get rid of the )...
					//	strcpy(portNamesShort[nPorts], begin);
					//}
					//if (nPorts++ > MAX_SERIAL_PORTS) {}

				}
			}
			i++;
		}
	}
	SetupDiDestroyDeviceInfoList(hDevInfo);

	bPortsEnumerated = false;
}


//---------------------------------------------
#endif
//---------------------------------------------