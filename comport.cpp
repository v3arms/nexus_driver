#include "comport.h"


/*
int openAsComPort(const char* device, int _vtime, int _vmin, int _baudrate) {
    int fd;
    struct termios tio;

    fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(device);
        return -1;
    }
    
    // tcgetattr(fd, &tio);
    bzero(&tio, sizeof(tio));

    tio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    tio.c_cc[VTIME] = _vtime;
    tio.c_cc[VMIN]  = _vmin;

    tcflush(fd, TCIFLUSH); // ? flush some received data
    tcsetattr(fd, TCSANOW, &tio);

    return fd;
} */


int openAsComPort(const char* device, int _vtime, int _vmin, int _baudrate) {
    errno = 0;
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("%s\n", strerror(errno));
        return -1;
    }
    
    if(flock(fd, LOCK_EX | LOCK_NB) == -1) {
       printf("Cannot use comport : locked by other process");
       return -1;
    }
    
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        printf("%s\n", strerror(errno));
        return -1;
    }
    memset(&tty, 0, sizeof(tty));
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag  = CREAD | CLOCAL;
    tty.c_cflag |= CS8;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // tty.c_iflag = IGNPAR;
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    // tty.c_oflag = 0;

    tty.c_cc[VTIME] = _vtime;
    tty.c_cc[VMIN]  = _vmin;

    cfsetispeed(&tty, _baudrate);
    cfsetospeed(&tty, _baudrate);

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }   

    return fd;
}
