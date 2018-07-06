/****************************************
      Date:2018-06-14
      Author:Peakzuo
      Decribe:AGV_Adjust
****************************************/
#include "../include/agv_control/agv_control.h"
#include<bitset>
using std::bitset;
using namespace AGV;

//constructor
Direction_PID::Direction_PID():KP(0),KI(0),KD(0),err_now(0),
    dCtrOut(0),ctrOut(0),err_next(0),err_last(0)
{
    if(!nh.getParam("KP",KP))
        KP = 1.0;
    if(!nh.getParam("KI",KI))
        KI = 0.1;
    if(!nh.getParam("KD",KD))
        KD = 0.2;
    if(!nh.getParam("linear_x",linear_x))
        linear_x = -0.1;

    sub = nh.subscribe("/mobile_base/sensors/core",100,&Direction_PID::Control_Callback,this);
    pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);

}

//pid
void Direction_PID::PID_control()
{
    err_now = delta_dist;

    dCtrOut = KP*(err_now-err_next) + KI*err_now
                 +KD*(err_now-2*err_next+err_last);

    err_last = err_next;
    err_next = err_now;

    if(KP==0&&KI==0&&KD==0) ctrOut = 0;
    else ctrOut += dCtrOut;
}


void Direction_PID::Control_Callback(const kobuki_msgs::SensorStateConstPtr &core_data)
{
     static bool stop_sate=false;
     static uint16_t i=0,j=0;
     static uint16_t tmp[15]={0};
     velocity.linear.x=0;
     velocity.angular.z=0;
     size_t count=0;
     delta_dist=0;
     CurValue=core_data->bottom[0];

     bitset<16> bit(CurValue);
     count = bit.count();
     ROS_INFO("number=%lu",count);

     for(int i=0;i<15;i++)
     {
        if(bit.test(i)==true){
          delta_dist = (i+1-7)*0.01;
          if(3==count)
              delta_dist = (i+1-7)*0.01+0.005;
          break;
        }
        else
           delta_dist = 8*0.01;
     }
      PID_control();
      velocity.linear.x = linear_x;
      if(delta_dist<=0)
         velocity.angular.z = 2*fabs(ctrOut);
      else if(delta_dist>0)
         velocity.angular.z = -2*fabs(ctrOut);
      ROS_INFO("ctrout=%f",ctrOut);

      if(velocity.angular.z>1) velocity.angular.z = 1;
      if(velocity.angular.z<-1) velocity.angular.z = -1;

      tmp[i++]=CurValue;      //accept a lot of 0,then stop
      if(i==4){
        for(i=0;i<4;i++)
        {
          if(tmp[i]==0)
          j++;
        }
        if(j==4){
          velocity.linear.x=0;
          velocity.angular.z=0;
          pub.publish(velocity);
          stop_sate = true;
        }
        i=0;
        j=0;
      }
      if(stop_sate==false)
       pub.publish(velocity);
      ROS_INFO("w=%f,dist=%f",velocity.angular.z,delta_dist);
}


int main(int argc,char**argv)
{
    ros::init(argc,argv,"agv_control");

    Direction_PID alignment;

    ros::spin();

    return 0;
}
