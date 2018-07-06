#include<ros/ros.h>
#include<geometry_msgs/Twist.h>


int main(int argc,char **argv)
{
   ros::init(argc,argv,"send_velocity");
   ros::NodeHandle node;


   geometry_msgs::Twist velocity;

   ros::Publisher CmdValue = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);

   while(ros::ok()){

     velocity.linear.x = 0.3;
     velocity.angular.z = 1;
     CmdValue.publish(velocity);
     ROS_INFO("linear.x=%f,angular.z=%f",velocity.linear.x, velocity.angular.z);
   }

   return 0;
}
