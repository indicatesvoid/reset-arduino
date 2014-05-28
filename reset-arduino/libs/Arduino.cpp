//
//  Arduino.cpp
//  Trenchcoat
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

bool Arduino::connect(int baud = 9600) {
	bInited = false;

    if(arduinos.size() == 0) cout << "No Arduinos found. Please plug one in and run the application again." << endl;
    else if(arduinos.size() > 1) device_path = chooseActiveDevice();
    else device_path = arduinos[0];
    
	//---------------------------------------------
	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
	//---------------------------------------------

		// open the serial device
		fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
    
		if(fd == -1){
			std::cout << "Unable to open port\n";
			return false;
		}
    
		// set baud rate
		struct termios ComParams;
		tcgetattr(fd, &ComParams);
		ComParams.c_cflag &= baud;
		ComParams.c_cflag |= B9600;
		tcsetattr( fd, TCSANOW, &ComParams );
    
		std::cout << "Success in opening serial port\n";
		bInited = true;
		return true;

	//---------------------------------------------
	#endif
	//---------------------------------------------
	
	//---------------------------------------------
	#ifdef TARGET_WIN32
	//---------------------------------------------
		hComm = CreateFileA(device_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

		if (hComm == INVALID_HANDLE_VALUE) {
			std::cout << "Unable to open port\n";
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
				std::cout << "ERROR: Something went wrong with the serial connection. Insert unhelpful error message here.";
			}
		#else
		if (!BuildCommDCB(buf, &cfg.dcb)){
			std::cout << "Something went wrong with the serial connection. Insert unhelpful error message here.";
		}
		#endif

		if (!SetCommState(hComm, &cfg.dcb)){
			std::cout << "ERROR: Can't set comm state";
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

		std::cout << "Success in opening serial port\n";
		bInited = true;
		return true;
	//---------------------------------------------
	#endif
	//---------------------------------------------
}

void Arduino::close(){

	//---------------------------------------------
#ifdef TARGET_WIN32
	//---------------------------------------------
	if (bInited){
		SetCommTimeouts(hComm, &oldTimeout);
		CloseHandle(hComm);
		hComm = INVALID_HANDLE_VALUE;
		bInited = false;
	}
	//---------------------------------------------
#else
	//---------------------------------------------
	if (bInited){
		tcsetattr(fd, TCSANOW, &oldoptions);
		::close(fd);
	}
	// [CHECK] -- anything else need to be reset?
	//---------------------------------------------
#endif
	//---------------------------------------------

}

void Arduino::reset() {
    // set a high bit over the serial line, wait, then reset
    // this will smack the Arduino into resetting
    std::cout<<"Resetting Arduino..." << std::endl;
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

//////////////////////////////////////////////
//                                          //
//              PRIVATE METHODS             //
//                                          //
//////////////////////////////////////////////

void Arduino::listDetectedDevices() {
    for(int i = 0; i < arduinos.size(); i++) {
        string index = to_string(i);
        cout << "[" << index << "] " << arduinos[i] << endl;
    }
}

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
    cout << "Multiple Arduinos detected:/n";
    listDetectedDevices();
    cout << "Please choose active device: ";
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
        cout << "Selection invalid. Please make sure input is a valid port index or path.";
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
    
		if (dir == NULL) std::cout << "Error listing devices in /dev";
		
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