//
//  main.cpp
//  reset-arduino
//
//  Created by William Clark on 1/15/14.
//  Copyright (c) 2014 William Clark. All rights reserved.
//

#include <unistd.h>
#include <stdio.h>

#include "Arduino.h"

Arduino* arduino = new Arduino();

int main(int argc, char **argv)
{
    int opt = 0;
    int board = OTHER;
    
    const char *opts = "b:";
    
    while ((opt = getopt(argc, argv, opts)) != -1) {
        switch(opt) {
            case 'b':
                if(strcmp(optarg, "leonardo") == 0|| strcmp(optarg, "leo") == 0) {
                    std::cout << "Leonardo" << "\n" << std::endl;
                    board = LEONARDO;
                }
                else board = OTHER;
                break;
        }
    }
    
    // connect at 9600 baud
//    arduino->connect(9600);
    
    // attempt reset
    arduino->reset(board);
    
    // delete references
    delete arduino;

    return 0;
}

