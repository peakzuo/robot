#ifndef SERIAL_H
#define SERIAL_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <malloc.h>
#include <termios.h>
#include "math.h"
#include <stdbool.h>
#include <sys/time.h>
#include <ros/ros.h>
class uart_driver{
 public:
   int OpenSerial(unsigned int baundrate);
   int Puts(unsigned char *data,int len);
   int Gets(unsigned char *data,int len);
    void CloseSerial(void);
};

#endif // SERIAL_H
