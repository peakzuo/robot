#include "../include/leishen_node/leishen_diff.h"
#include <pthread.h>

//static pthread_mutex_t state_mutex= PTHREAD_MUTEX_INITIALIZER;
//static pthread_mutex_t velocity_mutex = PTHREAD_MUTEX_INITIALIZER;

extern double  Get_yaw;

DiffDrive::DiffDrive() :
  last_velocity_left(0.0),
  last_velocity_right(0.0),
  last_tick_left(0),
  last_tick_right(0),
  last_rad_left(0.0),
  last_rad_right(0.0),
//  v(0.0), w(0.0), // command velocities, in [m/s] and [rad/s]
  radius(0.0), speed(0.0), // command velocities, in [mm] and [mm/s]
  point_velocity(2,0.0), // command velocities, in [m/s] and [rad/s]
  bias(0.258), // wheelbase, wheel_to_wheel, in [m]
  wheel_radius(0.030), // radius of main wheel, in [m]
  tick_to_rad(0.00872664611111111f)
{}



Pose_2d DiffDrive::Get_Pose_2d(const double &dleft,const double &dright)
{
 
  double sub =dright-dleft;
  static double delta_x,delta_y,x,y;
  static Pose_2d Get_pose;
  double ds = wheel_radius*(dleft+dright)/2.0;
  double domega = wheel_radius*(dright-dleft)/bias;
  delta_x = ds*cos(Get_pose.yaw+domega);
  delta_y = ds*sin(Get_pose.yaw+domega);
  Get_pose.delta_yaw = domega;
 
  Get_pose.delta_x = ds*cos(Get_yaw);
  Get_pose.delta_y = ds*sin(Get_yaw);
  
  x += delta_x;
  y += delta_y;
  
  Get_pose.x += Get_pose.delta_x;
  Get_pose.y += Get_pose.delta_y;
  Get_pose.yaw += Get_pose.delta_yaw;
 
  //ROS_INFO("sub=%f,left=%f,right=%f",sub,dleft,dright);
  //ROS_INFO("cos=%f,sin=%f,domega=%f",cos(domega),sin(domega),domega);
  //ROS_INFO("x=%f,y=%f,x_rate=%f,y_rate=%f",Get_pose.x,Get_pose.y,Get_pose.delta_x,Get_pose.delta_y);
  //ROS_INFO("x_heading=%f,y_heading=%f,x=%f,y=%f",Get_pose.x,Get_pose.y,x,y);
  //ROS_INFO("odom_to_yaw=%f,yaw=%f",Get_pose.yaw,Get_yaw);
  return Get_pose;
}


/**
 * @brief Updates the odometry from firmware stamps and encoders.
 *
 * Really horrible - could do with an overhaul.
 *
 * @param time_stamp
 * @param left_encoder
 * @param right_encoder
 * @param pose_update
 * @param pose_update_rates
 */
void DiffDrive::update(const uint16_t &time_stamp,
                       const uint16_t &left_encoder,
                       const uint16_t &right_encoder,
                       Pose_2d &pose_update,
                       Pose_3d &pose_update_rates) {
  //state_mutex.lock();
//  pthread_mutex_lock(&state_mutex);
  static bool init_l = false;
  static bool init_r = false;
  double left_diff_ticks = 0.0f;
  double right_diff_ticks = 0.0f;
  unsigned short curr_tick_left = 0;
  unsigned short curr_tick_right = 0;
  unsigned short curr_timestamp = 0;
  curr_timestamp = time_stamp;
  curr_tick_left = left_encoder;
  if (!init_l)
  {
    last_tick_left = curr_tick_left;
    init_l = true;
  }
  
  left_diff_ticks = (double)(short)((curr_tick_left - last_tick_left) & 0xffff);
  last_tick_left = curr_tick_left;
  last_rad_left += tick_to_rad * left_diff_ticks;
 
  curr_tick_right = right_encoder;
  if (!init_r)
  {
    last_tick_right = curr_tick_right;
    init_r = true;
  }
  right_diff_ticks = (double)(short)((curr_tick_right - last_tick_right) & 0xffff);
  last_tick_right = curr_tick_right;
  last_rad_right += tick_to_rad * right_diff_ticks;

/***
  if(left_diff_ticks!=0||right_diff_ticks!=0)
  ROS_INFO("left_diff=%f,right_diff=%f,left_encoder=%d,right_encoder=%d",left_diff_ticks,right_diff_ticks,left_encoder,right_encoder);
****/

  // TODO this line and the last statements are really ugly; refactor, put in another place
  pose_update = Get_Pose_2d(tick_to_rad * left_diff_ticks, tick_to_rad * right_diff_ticks);

  if (curr_timestamp != last_timestamp)
  {
    last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
    last_timestamp = curr_timestamp;
    last_velocity_left = (tick_to_rad * left_diff_ticks) / last_diff_time;
    last_velocity_right = (tick_to_rad * right_diff_ticks) / last_diff_time;
  } else {
    // we need to set the last_velocity_xxx to zero?
  }
/*
  pose_update_rates << pose_update.x()/last_diff_time,
                       pose_update.y()/last_diff_time,
                       pose_update.heading()/last_diff_time;
*/
  pose_update_rates.x_rate = pose_update.delta_x/last_diff_time;
  pose_update_rates.y_rate = pose_update.delta_y/last_diff_time;
  pose_update_rates.yaw_rate = pose_update.delta_yaw/last_diff_time;

  //state_mutex.unlock();
//  pthread_mutex_unlock(&state_mutex);
}

void DiffDrive::reset() {
  //state_mutex.lock();
//  pthread_mutex_lock(&state_mutex);
  last_rad_left = 0.0;
  last_rad_right = 0.0;
  last_velocity_left = 0.0;
  last_velocity_right = 0.0;
  //state_mutex.unlock();
//  pthread_mutex_unlock(&state_mutex);
}

void DiffDrive::getWheelJointStates(double &wheel_left_angle, double &wheel_left_angle_rate,
                                    double &wheel_right_angle, double &wheel_right_angle_rate) {
  //state_mutex.lock();
//  pthread_mutex_lock(&state_mutex);
  wheel_left_angle = last_rad_left;
  wheel_right_angle = last_rad_right;
  wheel_left_angle_rate = last_velocity_left;
  wheel_right_angle_rate = last_velocity_right;
  //state_mutex.unlock();
//  pthread_mutex_unlock(&state_mutex);
}

void DiffDrive::setVelocityCommands(const double &vx, const double &wz) {
  // vx: in m/s
  // wz: in rad/s
  std::vector<double> cmd_vel;
  cmd_vel.push_back(vx);
  cmd_vel.push_back(wz);
  point_velocity = cmd_vel;
}

void DiffDrive::velocityCommands(const double &vx, const double &wz) {
  // vx: in m/s
  // wz: in rad/s
  //velocity_mutex.lock();
//  pthread_mutex_lock(&velocity_mutex);
  const double epsilon = 0.0001;

  // Special Case #1 : Straight Run
  if( std::abs(wz) < epsilon ) {
    radius = 0.0f;
    speed  = 1000.0f * vx;
    //velocity_mutex.unlock();
 //   pthread_mutex_unlock(&velocity_mutex);
    return;
  }

  radius = vx * 1000.0f / wz;
  // Special Case #2 : Pure Rotation or Radius is less than or equal to 1.0 mm
  if( std::abs(vx) < epsilon || std::abs(radius) <= 1.0f ) {
    speed  = 1000.0f * bias * wz / 2.0f;
    radius = 1.0f;
    //velocity_mutex.unlock();
 //   pthread_mutex_unlock(&velocity_mutex);
    return;
  }

  // General Case :
  if( radius > 0.0f ) {
    speed  = (radius + 1000.0f * bias / 2.0f) * wz;
  } else {
    speed  = (radius - 1000.0f * bias / 2.0f) * wz;
  }
  //velocity_mutex.unlock();
//  pthread_mutex_unlock(&velocity_mutex);
  return;
}

void DiffDrive::velocityCommands(const short &cmd_speed, const short &cmd_radius) {
  //velocity_mutex.lock();
//  pthread_mutex_lock(&velocity_mutex);
  speed = static_cast<double>(cmd_speed);   // In [mm/s]
  radius = static_cast<double>(cmd_radius); // In [mm]
  //velocity_mutex.unlock();
 // pthread_mutex_unlock(&velocity_mutex);
  return;
}

std::vector<short> DiffDrive::velocityCommands() {
  //velocity_mutex.lock();
//  pthread_mutex_lock(&velocity_mutex);
  std::vector<short> cmd(2);
  cmd[0] = bound(speed);  // In [mm/s]
  cmd[1] = bound(radius); // In [mm]
  //velocity_mutex.unlock();
//  pthread_mutex_unlock(&velocity_mutex);
  return cmd;
}

std::vector<double> DiffDrive::pointVelocity() const {
  return point_velocity;
}

short DiffDrive::bound(const double &value) {
  if (value > static_cast<double>(SHRT_MAX)) return SHRT_MAX;
  if (value < static_cast<double>(SHRT_MIN)) return SHRT_MIN;
  return static_cast<short>(value);
}

DiffDrive diff_drive;
//Demand_data demand_data;


void DiffDrive::updateOdometry(Pose_2d &pose_update, Pose_3d &pose_update_rates)
{
  diff_drive.update(demand_data.Timestamp, demand_data.Left_encoder, demand_data.Right_encoder,
                      pose_update, pose_update_rates);

}


void DiffDrive::sendBaseControlCommand(short &speed,short &radius)
{
  std::vector<double> velocity_commands_received;
  velocity_commands_received=diff_drive.pointVelocity();

  diff_drive.velocityCommands(velocity_commands_received);
  std::vector<short> velocity_commands = diff_drive.velocityCommands();
  speed = velocity_commands[0];
  radius = velocity_commands[1];
  // std::cout << "speed: " << velocity_commands[0] << ", radius: " << velocity_commands[1] << std::endl;
  //sendCommand(Command::SetVelocityControl(velocity_commands[0], velocity_commands[1]));

  /*
  //experimental; send raw control command and received command velocity
  velocity_commands_debug=velocity_commands;
  velocity_commands_debug.push_back((short)(velocity_commands_received[0]*1000.0));
  velocity_commands_debug.push_back((short)(velocity_commands_received[1]*1000.0));
 //sig_raw_control_command.emit(velocity_commands_debug);
  */
}
