/****************************************
      Date:2018-06-08
      Author:Peakzuo
      Decribe:Draw a square in map
****************************************/
#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/Twist.h>
#include<math.h>
#include<tf/LinearMath/Transform.h>
#include<tf/transform_listener.h>
int main(int argc,char **argv)
{
   ros::init(argc,argv,"darw");

   ros::NodeHandle nh;

   ros::Publisher PubValue= nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi",50);

   ros::Rate loop_rate(10);

   geometry_msgs::Twist Velocity;


   int step =0;

    while(nh.ok()){

    switch(step++){

     tf::TransformListener listener;
     tf::StampedTransform transform;

     listener.lookupTransform("base_footprint","odom",ros::Time(0),transform);

     transform.getRotation();

     case 0:
       Velocity.linear.x= 10;
       Velocity.angular.z=0;
        break;
     case 1:
       Velocity.linear.x=0;
       Velocity.angular.z=0.3;
        break;
      case 2:
     }

    //ros::spinOnce();
    loop_rate.sleep();

   }


}







