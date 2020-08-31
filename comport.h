#pragma once


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
// #include <strings.h>
#include <errno.h>
#include <string.h>
#include <sys/file.h>


const int BAUDRATE = 57600;


int openAsComPort(const char* device, int _vtime = 10, int _vmin = 4, int _baudrate = BAUDRATE);

