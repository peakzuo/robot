#ifndef AGV_CONTROL_H
#define AGV_CONTROL_H
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<kobuki_msgs/SensorState.h>
#include<math.h>

namespace AGV {
class Direction_PID{
   public:
   Direction_PID();  //constructor
   //PID相关的值
   float  KP;   //比例系数
   float  KI;   //积分系数
   float  KD;   //微分系数
   float  linear_x; //线速度

   uint16_t CurValue; //检测到磁轨的值
   float delta_dist;  //偏离的距离，单位为m

   float err_now;   //当前的误差
   float dCtrOut;  //控制增量输出
   float ctrOut;   //控制输出
   /**PID算法内部变量，其值不能修改**/
   float err_next;
   float err_last;

   ros::NodeHandle nh;
   ros::Subscriber sub;
   ros::Publisher pub;
   geometry_msgs::Twist velocity;
   void PID_control();
   void Control_Callback(const kobuki_msgs::SensorStateConstPtr &core_data);
   };
}
#endif
