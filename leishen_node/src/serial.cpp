#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>


#include <semaphore.h>
#include <pthread.h>

#include "../include/leishen_node/serial.h"

static int m_dFd;

int uart_driver::OpenSerial(unsigned int baudrate)
{
    struct termios  m_stNew;
    struct termios  m_stOld;

    const char *addr0 = "/dev/ttyS1";
    const char *addr1 = "/dev/ttyS1";

    //const char *add0 = "/dev/ttyUSB0";
    //const char *addr1 = "/dev/ttyUSB0";

    m_dFd = open(addr0, O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == m_dFd)
    {
        ROS_INFO("Open Serial Port Error!\n");
    m_dFd = open(addr1, O_RDWR|O_NOCTTY|O_NDELAY);
    if (m_dFd < 0)
        ROS_INFO("Open Serial Port Error!\n");
              return -1;
    }

    if( (fcntl(m_dFd, F_SETFL, 0)) < 0 )
    {
        perror("Fcntl F_SETFL Error!\n");
        return -1;
    }
    if(tcgetattr(m_dFd, &m_stOld) != 0)
    {
        perror("tcgetattr error!\n");
        return -1;
    }

    ROS_INFO("Open Serial Port Ok!\n");
    m_stNew = m_stOld;
    cfmakeraw(&m_stNew);

    //set speed
    cfsetispeed(&m_stNew, baudrate);//115200
    cfsetospeed(&m_stNew, baudrate);

    //set databits
    m_stNew.c_cflag |= (CLOCAL|CREAD);
    m_stNew.c_cflag &= ~CSIZE;
    m_stNew.c_cflag |= CS8;

    //set parity
    m_stNew.c_cflag &= ~PARENB;
    m_stNew.c_iflag &= ~INPCK;

    //set stopbits
    m_stNew.c_cflag &= ~CSTOPB;
    m_stNew.c_cc[VTIME]=0;
    m_stNew.c_cc[VMIN]=1;

    tcflush(m_dFd,TCIFLUSH);
    if( tcsetattr(m_dFd,TCSANOW,&m_stNew) != 0 )
    {
        perror("tcsetattr Error!\n");
        return -1;
    }

    return m_dFd;
}


int uart_driver::Puts(unsigned char *data, int len)
{
  int ret;

  ret = write(m_dFd, data, len);
  return ret;
}

int uart_driver::Gets(unsigned char *data, int len)
{
   int count = 0; 
   int pos = 0;
  
   read(m_dFd,data,2);
   if(data[0]==0xaa&&data[1]==0x55)  
    do{
      count =read(m_dFd,&data[pos+2],len-pos-2);
    
      if(0>count){
        if(EINTR==errno) continue;
        return(pos);   
       }
       if(0==count) break;
        pos += count;
      // ROS_INFO("count=%d",count);
     }while(len>pos+2);

/*
 if(pos+2==len){
  ROS_INFO("pos=%d,len=%d",pos+2,len);
  ROS_INFO("data[0]=%x,data[1]=%x,data[2]=%x",data[0],data[1],data[2]);
 }
*/
  return (pos+2);
}

void uart_driver::CloseSerial(void)
{
  close(m_dFd);
}





