/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef LEISHEN_ODOMETRY_H
#define LEISHEN_ODOMETRY_H



/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
//#include <ecl/geometry/legacy_pose2d.hpp>
/*****************************************************************************
** Namespaces
*****************************************************************************/



//unsigned char *pbuffer;

class Pose_2d{

public:
  double delta_x;
  double delta_y;
  double delta_yaw;
  double x;
  double y;
  double yaw;

};

class Pose_3d{

 public:
  double x_rate;
  double y_rate;
  double yaw_rate;

};

//Pose_2d operator*=(Pose_2d pose){
//  *this = (*this)*pose;
//  return (*this);
//}

/*****************************************************************************
** Interfaces
*****************************************************************************/
/**
 * @brief  Odometry for the leishen node.
 **/
class Odometry {
public:
  Odometry();
  void init(ros::NodeHandle& nh, const std::string& name);
  bool commandTimeout() const;
/*  void update(const ecl::LegacyPose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
              double imu_heading, double imu_angular_velocity);
 */
  void update(const Pose_2d &pose_update, Pose_3d &pose_update_rates,double imu_heading, double imu_angular_velocity);
 // void resetOdometry() { pose.setIdentity(); }
  const ros::Duration& timeout() const { return cmd_vel_timeout; }
  void resetTimeout() { last_cmd_time = ros::Time::now(); }

  double getHeading(double &angle,double &initial_angle);
  double getAngularVelocity(double &angle_rate);

  double heading_offset;

private:
  geometry_msgs::TransformStamped odom_trans;
  //ecl::LegacyPose2D<double> pose;
  Pose_2d pose;
  std::string odom_frame;
  std::string base_frame;
  ros::Duration cmd_vel_timeout;
  ros::Time last_cmd_time;
  //bool publish_tf;
  //bool use_imu_heading;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_publisher;



  void publishTransform(const geometry_msgs::Quaternion &odom_quat);
  //void publishOdometry(const geometry_msgs::Quaternion &odom_quat, const ecl::linear_algebra::Vector3d &pose_update_rates);
  void publishOdometry(const geometry_msgs::Quaternion &odom_quat,const Pose_3d &pose_update_rates);
};


#define pi 3.1415926



#endif // LEISHEN_ODOMETRY_H
