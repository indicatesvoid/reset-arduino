//
//  main.cpp
//  reset-arduino
//
//  Created by William Clark on 1/15/14.
//  Copyright (c) 2014 William Clark. All rights reserved.
//

#include <stdio.h>
#include "Arduino.h"

Arduino* arduino = new Arduino();

int main(int argc, const char * argv[])
{
    // connect at 9600 baud
    arduino->connect(9600);
    
    // attempt reset
    arduino->reset();

    // insert code here...
    // printf("Hello, World!\n");
    return 0;
}

