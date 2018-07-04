/**
 * @file /leishen_node/src/leishen_odometry.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 **/

#include "../include/leishen_node/leishen_odometry.h"


double Get_yaw;

Odometry::Odometry () :
  odom_frame("odom"),
  base_frame("base_footprint")
  //use_imu_heading(true),
  //publish_tf(true)
{}



void Odometry::init(ros::NodeHandle& nh, const std::string& name) {
  double timeout;
  //nh.param("cmd_vel_timeout", timeout, 0.6);
  //cmd_vel_timeout.fromSec(timeout);
  //ROS_INFO_STREAM("leishen : Velocity commands timeout: " << cmd_vel_timeout << " seconds [" << name << "].");


  odom_trans.header.frame_id = odom_frame;
  odom_trans.child_frame_id = base_frame;

  //pose.setIdentity();

  odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 50); // topic name and queue size
}

bool Odometry::commandTimeout() const {
  if ( (!last_cmd_time.isZero()) && ((ros::Time::now() - last_cmd_time) > cmd_vel_timeout) ) {
    return true;
  } else {
    return false;
  }
}

void Odometry::update(const Pose_2d &pose_update, Pose_3d &pose_update_rates,
                      double imu_heading, double imu_angular_velocity) {

  pose = pose_update;
  
  Get_yaw = imu_heading;  
  
  pose_update_rates.yaw_rate =  imu_angular_velocity;
 
  //since all ros tf odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_heading);
  
  //ROS_INFO("yaw=%f",imu_heading);
  if ( ros::ok() ) {
    publishTransform(odom_quat);
    publishOdometry(odom_quat, pose_update_rates);
  }
}

/*****************************************************************************
** Private Implementation
*****************************************************************************/

void Odometry::publishTransform(const geometry_msgs::Quaternion &odom_quat)
{
  //if (publish_tf == false)
    //return;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = pose.x;
  odom_trans.transform.translation.y = pose.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);

}

void Odometry::publishOdometry(const geometry_msgs::Quaternion &odom_quat,
                               const Pose_3d &pose_update_rates)
{
  // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

  // Header
  odom->header.stamp = ros::Time::now();
  odom->header.frame_id = odom_frame;
  odom->child_frame_id = base_frame;

  // Position
  odom->pose.pose.position.x = pose.x;
  odom->pose.pose.position.y = pose.y;
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation = odom_quat;

  // Velocity
  odom->twist.twist.linear.x = pose_update_rates.x_rate;
  odom->twist.twist.linear.y = pose_update_rates.y_rate;
  odom->twist.twist.angular.z = pose_update_rates.yaw_rate;

  // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
  // Odometry yaw covariance must be much bigger than the covariance provided
  // by the imu, as the later takes much better measures
  odom->pose.covariance[0]  = 0.1;
  odom->pose.covariance[7]  = 0.1;
  //odom->pose.covariance[35] = use_imu_heading ? 0.05 : 0.2;
  odom->pose.covariance[35] = 0.05 ;

  odom->pose.covariance[14] = DBL_MAX; // set a non-zero covariance on unused
  odom->pose.covariance[21] = DBL_MAX; // dimensions (z, pitch and roll); this
  odom->pose.covariance[28] = DBL_MAX; // is a requirement of robot_pose_ekf

  odom_publisher.publish(odom);
}



double wrap_angle(const double &angle)
{
   double wrapped;
   if(angle <= pi && angle >= -pi){
       wrapped = angle;
   }
   else if( angle < 0.0){
       wrapped = fmod(angle-pi,2.0*pi)+pi;
   }
   else{
       wrapped = fmod(angle+pi,2.0*pi)-pi;
   }
   return wrapped;
}


double Odometry::getHeading(double &angle,double &initial_angle)
{
  double heading;
  // raw data angles are in hundredths of a degree, convert to radians.
  heading = (static_cast<double>(angle) / 100.0) * pi / 180.0;
  return wrap_angle(heading - initial_angle);
}

double Odometry::getAngularVelocity(double &angle_rate)
{
  // raw data angles are in hundredths of a degree, convert to radians.
  return (static_cast<double>(angle_rate) / 100.0) * pi / 180.0;
}





