#ifndef PACKAGE_ANALYSIS_H
#define PACKAGE_ANALYSIS_H

#include<ros/ros.h>
#include"serial.h"
#include<termios.h>
class Demand_data{
public:
  unsigned short Timestamp;
  unsigned short Left_encoder;
  unsigned short Right_encoder;
  unsigned char Right_signal;
  unsigned char Central_signal;
  unsigned char Left_signal;
  //unsigned short Angle;
  //unsigned short Angle_rate;

  short Angle;
  short Angle_rate;
};

extern Demand_data demand_data;



class package_manage{

  public :
  int Init_serial(void);
  int GetPackage(unsigned char *data,unsigned int len);
  bool Data_analysis(unsigned char *tmp,Demand_data *Effective_Data);
  int SendData(short speed,short radius);
};

extern package_manage package_Get;


#endif // PACKAGE_ANALYSIS_H
