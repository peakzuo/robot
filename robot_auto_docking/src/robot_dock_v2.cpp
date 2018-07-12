/****************************************
      Date:2018-07-10
      Author:Peakzuo
      Decribe:ROBOT_DOCKING_V2
****************************************/
#include "../include/robot_auto_docking/robot_dock_v2.hpp"
using namespace robot;

//constructor
Robot_dock::Robot_dock():vx(0),vw(0)
{
   new_state = Init_state;
   sub = nh.subscribe("/mobile_base/sensors/dock_ir",10,&Robot_dock::docking_callback,this);
   pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);
}

void Robot_dock::operation_dock()
{
  switch(new_state){
    case Left_near:
         vx=0;
         vw=-0.1;
      break;
    case Left_far:
          vx=0.06;
          vw=-0.2;
      break;
    case Right_near:
          vx=0;
          vw=0.1;
      break;
    case Right_far:
          vx=0.06;
          vw=0.2;
      break;
    case Calibration_centrel:
          vx= 0.03;
          vw= 0;
      break;
    case inverse_left:
    case inverse_right:
         vx= 0.03;
         vw=0;
      break;
    case Suspend:
         vx= 0;
         vw= 0;
      break;
  }

  velocity.linear.x=vx;
  velocity.angular.z=vw;
  pub.publish(velocity);
  ROS_INFO("vx=%f,vw=%f",vx,vw);
  pre_state=new_state;
}

void Robot_dock::docking_callback(const kobuki_msgs::DockInfraRed &IR_Value)
{

  uint8_t left_received=0,centrel_received=0,right_received=0;
  right_received = IR_Value.data[0]&0x0d;
  centrel_received = IR_Value.data[1]&0x0d;
  left_received = IR_Value.data[2]&0x0d;


  if((left_received&0x04)&&((right_received&0x01)==0)&&right_received!=0)
      new_state= Left_near;

  else if(left_received==0&&right_received==0x04)
      new_state= Left_far;

  else if((right_received&0x01)&&((left_received&0X04)==0)&&left_received!=0)
      new_state= Right_near;

  else if(right_received==0&&left_received==0x01)
      new_state= Right_far;

  else if((right_received&0X01)&&(left_received&0x04))
      new_state= Calibration_centrel;

  else if(left_received==0x00&&(right_received==0x01||right_received==0x05))
      new_state= inverse_left;

  else if(right_received==0x00&&left_received==0x04)
      new_state= inverse_right;

  else if(right_received==0&&left_received==0)
      new_state= Suspend;

  operation_dock();

  ROS_INFO("right=%d,left=%d,state=%d",right_received,left_received,new_state);
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"robot_docking");

    Robot_dock auto_dock;

    ros::spin();

}
