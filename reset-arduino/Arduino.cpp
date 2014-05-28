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
    bIsInited = bArduinoFound = false;
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
    // set baud rate
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
            std::cout << "Cannot set " << baud << " bps, setting to 9600";
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
    std::cout << "Setting baud rate " << to_string(retrievedBaud) << "\n";
    currentBaud = retrievedBaud;
}

bool Arduino::openPort() {
    bIsInited = false;
    
    // open the serial device
    fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
    
    if(fd == -1){
        std::cout << "Unable to open port\n";
        return false;
    }
    
    // safety - set baud rate once more after open
    setBaud((int)currentBaud);
    
    std::cout << "Success in opening serial port at " << device_path << std::endl;
    struct termios opts;
    tcgetattr(fd, &opts);
    unsigned long retrievedBaud = cfgetospeed(&opts);
    
    std::cout << "At baud rate " << to_string(retrievedBaud) << "\n";
    
    bIsInited = true;
    return true;
}

bool Arduino::connect(int baud, bool verbose) {
    find();
    setBaud(baud);
    bool success = openPort();
    
    return success;
}

void Arduino::closePort() {
    if(bIsInited) {
//        tcsetattr(fd,TCSANOW,&oldoptions);
        ::close(fd);
        
        std::cout << "Success in closing serial port at " << device_path << std::endl;
        
//        struct termios options;
//        tcgetattr(fd, &options);
//        options.c_cflag &= ~CRTSCTS; // RTS/CTS Flow Control
//        options.c_cflag &= ~(CDTR_IFLOW | CDSR_OFLOW); // DTR/DSR Flow Control
//        options.c_cflag &= ~CCAR_OFLOW; // DCD Flow Control
//        tcsetattr(fd, TCSANOW, &options);
//        
//        tcsetattr(fd, TCSADRAIN, &oldoptions);
//        fd = 0;
        
        
        bIsInited = false;
    }
    
    else std::cout << "Serial port at " << device_path << " is not open. Nothing to close." << std::endl;
}

void Arduino::reset(int _board) {
    
//    ser = serial.Serial(args.port[0], 57600)
//    ser.close()
//    ser.open()
//    ser.close()
//    ser.setBaudrate(1200)
//    ser.open()
//    ser.close()
//    sleep(1)
//    
//    while not os.path.exists(args.port[0]):
//        if args.verbose: print('Waiting for %s to come back' % args.port[0])
//            sleep(1)
//            
//            if args.verbose: print('%s has come back after reset' % args.port[0])
    if(!bArduinoFound) find();
    
    if(_board == LEONARDO) {
        std::cout<<"Resetting Leonardo..." << std::endl;
        // force reset using 1200bps open/close
        setBaud(57600);
        closePort();
        openPort();
        closePort();
        
        setBaud(1200);
        openPort();
        closePort();
        sleep(1);
    }
    
    else {
        connect();
        // set a high bit over the serial line, wait, then reset
        // this will smack the Arduino into resetting
        std::cout<<"Resetting Arduino..." << std::endl;
        ioctl(fd, TIOCMBIS, TIOCM_DTR);
        usleep(500);
        ioctl(fd, TIOCMBIC, TIOCM_DTR);
    }
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
    while (it != input.end() && std::isdigit(*it)) ++it;
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
    
    DIR *dir;
	struct dirent *entry;
	dir = opendir("/dev");
    
	string deviceName	= "";
    
	if (dir == NULL){
        std::cout << "Error listing devices in /dev";
	} else {
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
}