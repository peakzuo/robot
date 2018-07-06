#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<tf/tf.h>
#include"../include/laser_pose/laser_pose.hpp"
class Gmapping gmapping;
void  lasercallback(const sensor_msgs::LaserScan &scan)
{
  tf::TransformListener tf_;

  ROS_INFO("range=%f",scan.ranges[2]);
  gmapping.laser_frame_ = scan.header.frame_id;
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_=gmapping.laser_frame_;
  ident.stamp_= scan.header.stamp;
  try{
      ROS_ERROR_STREAM(gmapping.base_frame_);
      tf_.transformPose(gmapping.base_frame_,ident,laser_pose);
  }

  catch(tf::TransformException &e)
  {

      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",e.what());
     //return false;
  }

  //create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0,0,1+laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v,scan.header.stamp,gmapping.base_frame_);

  try
  {
      tf_.transformPoint(gmapping.laser_frame_,up,up);
	  ROS_DEBUG("Z-Axis in sensor frame:%.3f",up.z());
  }

  catch(tf::TransformException& e)
  {
       ROS_WARN("Unable to determine orientation of laser: %s",e.what());
       //return false;
  }

  if(fabs(fabs(up.z())-1)>0.001)
  {
	  ROS_WARN("Laser has to be mounted  planar ! Z-coordinate has to be 1 or -1,but gave:%.5f",up.z());
      //return false;
  }
    return ;
}


int main(int argc,char **argv)
{
  ros::init(argc,argv,"laser_pose");

  ros::NodeHandle nh;

  ros::Subscriber scan_sub=nh.subscribe("scan",100,lasercallback);

  if(!nh.getParam("odom_frame",gmapping.odom_frame_))
     gmapping.odom_frame_="odom";

  if(!nh.getParam("map_frame",gmapping.map_frame_))
     gmapping.map_frame_="map";

  if(!nh.getParam("base_frame",gmapping.base_frame_))
     gmapping.base_frame_="base_link";

   ros::Rate loop_rate(70);

   while(nh.ok())
   {
     ros::spinOnce();
     loop_rate.sleep();
   }
  return 0;
}
