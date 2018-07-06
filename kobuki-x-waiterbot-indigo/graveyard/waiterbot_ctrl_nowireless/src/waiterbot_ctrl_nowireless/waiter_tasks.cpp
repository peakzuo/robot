/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot
{

bool WaiterIsolated::recordOrderOrigin(std::string& message)
{
  // base frame is robot's pose on global frame
  tf::StampedTransform robot_tf;
  
  if(tf_handlers_.getTf(global_frame_, base_frame_,robot_tf) == false) 
  {
    std::stringstream sstm;
    sstm << "Failed to get transform between " << global_frame_ << " and " << base_frame_;
    message = sstm.str();
    return false;
  }

  mtk::tf2pose(robot_tf, robot_origin_);
  ROS_INFO("Waiter : Order received from %.2f %.2f %.2f", robot_origin_.pose.position.x, robot_origin_.pose.position.y, tf::getYaw(robot_origin_.pose.orientation));

  return true;
}

bool WaiterIsolated::goToVendingMachine(std::string& message)
{
  // go to in front of vending machine
  geometry_msgs::PoseStamped vm;// = map_wp_[loc_vm_];
  tf::StampedTransform vm_tf;
  // get target pose frame transform tree
  if(tf_handlers_.getTf(global_frame_, nav_target_vm_, vm_tf) == false)
  {
    std::stringstream sstm;
    sstm << "Failed to get transform between " << global_frame_ << " and " << base_frame_;
    message = sstm.str();
    return false;
  }
  mtk::tf2pose(vm_tf, vm);

  vm.header.stamp = ros::Time::now();
  navigator_.clearCostMaps();

  // go close to the VM using navigation
  ROS_INFO("Waiter : Navigating to vending machine ...");
  if(navigator_.moveTo(vm) == false) {
    ROS_ERROR("Navigation Failed while going to vending machine");
    message = "navigation failed....";
    return false;
  }

  // re-intialise
//  ROS_INFO("Waiter : Re-initialising before approach ...");
//  if(reinitialise() == false)
//  {
//    ROS_ERROR("Navigation failed while reinitialising.");
//    message = "navigation failed....";
//    return false;
//  }

  // approach the VM using the approach controller
  ROS_INFO("Waiter : Approaching vending machine ...");
  if(approachVM() == false)
  {
    ROS_ERROR("Navigation Failed while approaching the machine");
    message = "navigation failed....";
    return false;
  }

  playSound("pab.wav");

  return true;
}

bool WaiterIsolated::goToOrigin(std::string& message)
{
  // move back a bit
  navigator_.backward(0.3);
  navigator_.clearCostMaps();

  // get origin pose frame transform tree
  geometry_msgs::PoseStamped vm;
  tf::StampedTransform vm_tf;
  if(tf_handlers_.getTf(global_frame_, nav_target_origin_, vm_tf) == false)
  {
    std::stringstream sstm;
    sstm << "Failed to get transform between " << global_frame_ << " and " << base_frame_;
    message = sstm.str();
    return false;
  }
  mtk::tf2pose(vm_tf, vm);

  if(tray_empty_ == false) {
    if(navigator_.moveTo(vm) == false) {
      ROS_ERROR("Failed to move");
      message = "navigation failed...";
      return false;
    }
  }

  playSound("kaku.wav");
  return true;
}

bool WaiterIsolated::dockInBase() {
  kobuki_msgs::AutoDockingGoal ad_goal;

  in_docking_ = true;
  ac_autodock_.sendGoal(ad_goal);

  while (ac_autodock_.waitForResult(ros::Duration(3.0)) == false)
  {
    ROS_INFO("Waiter : Waiting for docking...");
  }

  in_docking_ = false;
  if(ac_autodock_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully docked...");
    return true;
  }
  else {
    ROS_ERROR("Failed to dock...");
    return false;
  }
}

void WaiterIsolated::playSound(const std::string& wav_file) {
  if ((wav_file.length() > 0))
    bool return_unused = system(("rosrun waiterbot_bringup play_sound.bash " + resources_path_ + wav_file).c_str());

}

/**
 * Commands the pose controller to move precisely in front of the VM
 *
 * @return true, if VM has been approached.
 */
bool WaiterIsolated::approachVM()
{
  // activate the pose controller
  std_msgs::Bool bool_msg;
  bool_msg.data = true;
  pub_pose_ctrl_enable_.publish(bool_msg);

  // wait for result
  while (!vm_approached_ && ros::ok())
  {
//    vm_approached_ = true; // for testing TODO: remove
    if (cancel_order_)
    {
      ROS_WARN("Waiter : Cancel command received during approach of the vending machine.");
      // disable the pose controller
      bool_msg.data = false;
      pub_pose_ctrl_enable_.publish(bool_msg);
      return false;
    }
    ros::Duration(0.1).sleep();
  }
  vm_approached_ = false;

  // disable the pose controller
  bool_msg.data = false;
  pub_pose_ctrl_enable_.publish(bool_msg);

  ROS_INFO("Waiter : VM successfully approached.");
  return true;
}

/**
 * Re-initialises the robot pose
 *
 * @return true, if re-initialisation succeeded.
 */
bool WaiterIsolated::reinitialise()
{
  // disable the pose controller
  std_msgs::Empty empty_msg;
  pub_initialise_pose_.publish(empty_msg);

  while (!pose_initialised_ && ros::ok())
  {
    if (cancel_order_)
    {
      ROS_WARN("Waiter : Cancel command received during re-initialisation");
      // disable the pose controller
      return false;
    }
    ros::Duration(0.1).sleep();
  }
  pose_initialised_ = false;

  ROS_INFO("Waiter : Successfully re-initialised.");
  return true;
}

} // namespace waiterbot

