#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
int main(int argc,char **argv)
{
  ros::init(argc,argv,"listener");

  ros::NodeHandle nh;

  tf::TransformListener listener;

  double x=0,y=0;

  std::string target_frame,source_frame;

  if(!nh.getParam("target_frame",target_frame))
      target_frame = "odom";
  if(!nh.getParam("source_frame",source_frame))
      source_frame = "base_link";


  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    tf::StampedTransform transform;
    try{
          listener.lookupTransform(target_frame,source_frame,ros::Time(0),transform);
       }

    catch(tf::TransformException &ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(10).sleep();
          continue;
      }

      x=transform.getOrigin().x();
      y=transform.getOrigin().y();

      ROS_INFO("x=%f,y=%f",x,y);

      loop_rate.sleep();
  }
   return 0;
}
