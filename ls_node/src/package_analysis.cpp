#include"../include/ls_node/package_analysis.h"

static uart_driver serial;
Demand_data demand_data;
package_manage package_Get;
unsigned char Serial_Buffer[87]={0};
int  package_manage::Init_serial(void)
{
     int ret;

    ret = serial.OpenSerial(B115200);

    if(ret<0)
    return ret;
  return ret;
}


int  package_manage::GetPackage(unsigned char *data,unsigned int len)
{
   
   return serial.Gets(data,len);

}

static unsigned char CRC_Verify(unsigned char *cBuffer,unsigned int len)
{

   unsigned char crc = 0;
   unsigned int i;


   for(i=0;i<len;i++)
     crc ^=cBuffer[i];
   return crc;

}


bool package_manage::Data_analysis(unsigned char *tmp,Demand_data *Effective_Data)
{
  unsigned char crc,*p;


  if(tmp[0]== 0xaa&&tmp[1]== 0x55&&tmp[2]== 0x53&&tmp[3]== 0x01&&tmp[4]== 0x0f&&tmp[20]== 0x03&&tmp[21]==0x03&&tmp[25]== 0x04&&tmp[26]== 0x07){
   p=tmp+2;
   crc=CRC_Verify(p,tmp[2]+1);

   if(crc == *(p+tmp[2]+1)){
     Effective_Data->Timestamp= (short)(tmp[5]|(tmp[6]<<8));
     Effective_Data->Left_encoder= (short)(tmp[10]|(tmp[11]<<8));
     Effective_Data->Right_encoder= (short)(tmp[12]|(tmp[13]<<8));
     Effective_Data->Right_signal = tmp[22];
     Effective_Data->Central_signal = tmp[23];
     Effective_Data->Left_signal = tmp[24];
     Effective_Data->Angle = (short)(tmp[27]|(tmp[28]<<8));
     Effective_Data->Angle_rate = (short)(tmp[29]|(tmp[30]<<8));
    //ROS_INFO("data2=%x,data3=%x,data4=%x,data5=%x,data68=%x,data69=%x,data86=%x,crc=%x",tmp[2],tmp[3],tmp[4],tmp[5],tmp[68],tmp[69],*(p+tmp[2]+1),crc);
    //if(Effective_Data->Left_encoder!=0||Effective_Data->Right_encoder!=0)
    //ROS_INFO("left=%d,right=%d",Effective_Data->Left_encoder,Effective_Data->Right_encoder);
    //ROS_INFO("Sign[0]=%x,Sign[1]=%x,YAW=%f,YAW_RATE=%f",tmp[25],tmp[26],Effective_Data->Angle/100.0,Effective_Data->Angle_rate/1.0); 
     return true;
    }

    else
    return false;

   }

}


int package_manage::SendData(short speed,short radius)
{
     unsigned char send_data[10]={0};
     unsigned char *p;
     send_data[0]= 0xaa;
     send_data[1]= 0x55;
     send_data[2]= 0x06;
     send_data[3]= 0x01;
     send_data[4]= 0x04;
     send_data[5]=speed&0xff;
     send_data[6]=(speed>>8)&0xff;
     send_data[7]=radius&0xff;
     send_data[8]=(radius>>8)&0xff;
     p = send_data+2;
     send_data[9]= CRC_Verify(p,send_data[2]+1);
  
  //ROS_INFO("speed=%d,radius=%d",speed,radius);
 
     return serial.Puts(send_data,10);

}

