/****************************************
      Date:2018-06-25
      Author:Peakzuo
      Decribe:AGV_ARM
****************************************/
#include"../include/agv_control/AGV_raise_and_down.h"
using namespace agv;

//constructor
Agv_sign::Agv_sign()
{
   sub_obst_sign = nh.subscribe("/mobile_base/sensors/core",100,&Agv_sign::obst_signcallback,this);
   sub_dest_sign = nh.subscribe("/agv_sign",100,&Agv_sign::dest_signcallback,this);
   pub= nh.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1",10);
}

void Agv_sign::obst_signcallback(const kobuki_msgs::SensorStateConstPtr &core_data)
{
    uint8_t obstacle_sign=0;
    obstacle_sign=core_data->cliff;
}

void Agv_sign::dest_signcallback(const std_msgs::Int16 &value)
{
    Oparetion_sign.value=0;

    if(value.data==Loading)
      Oparetion_sign.value = 1;
    else if(value.data==Unloading)
      Oparetion_sign.value = 2;
    else
      Oparetion_sign.value = 0;

    pub.publish(Oparetion_sign);
}



int main(int argc,char**argv)
{
    ros::init(argc,argv,"agv_updown");

    Agv_sign agv_state;

    ros::spin();

    return 0;
}
