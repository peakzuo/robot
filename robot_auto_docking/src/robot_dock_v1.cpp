/****************************************
      Date:2018-07-10
      Author:Peakzuo
      Decribe:ROBOT_DOCKING_V1(using)
****************************************/
#include "../include/robot_auto_docking/robot_dock_v1.hpp"
using namespace robot;

/**
 * @brief Constructor
**/
Robot_dock::Robot_dock():vx(0),vw(0),delta_vx(0),delta_vw(0),delta_vx1(0),count(0)
{
   new_state = Init_state;
   sub = nh.subscribe("/mobile_base/sensors/dock_ir",10,&Robot_dock::docking_callback,this);
   pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);
}

/**
 * @brief Dock operation
**/
void Robot_dock::operation_dock()
{
  switch(new_state){
    case Left_near:
         vx=0.05-delta_vx1;    //0
         vw=-0.2+delta_vw;   //-0.1
      break;
    case Left_far:
          vx=0.1-delta_vx;    //0.06
          vw=-0.25+delta_vw;   //-0.2
      break;
    case Right_near:
          vx=0.05-delta_vx1;     //0
          vw=0.2-delta_vw;   //0.1
      break;
    case Right_far:
          vx=0.1-delta_vx;   //0.06
          vw=0.25-delta_vw;   //0.2
      break;
    case  Centrel:
         vx= 0.1-delta_vx; //0.07
         vw= 0;
      break;
    case Rotation:
         vx= 0.1;
         vw= 0.7;
      break;
    case dock_near:
         delta_vx =0.05;
         delta_vx1=0.02;
         delta_vw=0.1;
         vx= 0.1-delta_vx; //0.07
         vw= 0;
        ROS_ERROR("close!!!");
     break;
  }

  velocity.linear.x=vx;
  velocity.angular.z=vw;
  pub.publish(velocity);
  ROS_INFO("vx=%f,vw=%f",vx,vw);
  //pre_state=new_state;
}

/**
 * @brief Judge robot in area
**/
void Robot_dock::docking_callback(const kobuki_msgs::DockInfraRed &IR_Value)
{

  uint8_t left_received=0,centrel_received=0,right_received=0;
  right_received = IR_Value.data[0]&0x0d;
  centrel_received = IR_Value.data[1]&0x0d;
  left_received = IR_Value.data[2]&0x0d;


  if((left_received==0x04)&&(right_received==0X05||right_received==0X04))
      new_state= Left_near;

  else if(left_received==0&&right_received!=0)
      new_state= Left_far;

  else if((right_received==0x01)&&(left_received==0x05||left_received==0x01))
      new_state= Right_near;

  else if(right_received==0&&left_received!=0)
      new_state= Right_far;

  else if((right_received==0X01)&&(left_received==0x04)){
      new_state= Centrel;
      count++;
  }

  else if(right_received==0&&left_received==0)
      new_state= Rotation;

  else if(count>=100)
      new_state= dock_near;

  operation_dock();

  ROS_INFO("right=%d,left=%d,state=%d,count=%d",right_received,left_received,new_state,count);
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"robot_docking");

    Robot_dock auto_dock;

    ros::spin();

}
