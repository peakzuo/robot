
//waypoint
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <yocs_msgs/WaypointList.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/NavigationControlStatus.h>
#include <yocs_msgs/NavigationControl.h>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <vector>
#include <queue>
#include <stack>
#include <map>
#include <cmath>
//导航命令

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt64MultiArray.h>

#define  Pi 3.141591653




static double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}
/*
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}
*/
static double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}
static double computeNewVelocity(double vg, double vi, double a_max, double dt){
  if((vg - vi) >= 0) {
    return std::min(vg, vi + a_max * dt);
  }
  return std::max(vg, vi - a_max * dt);
}

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;
  double sign0=sign(B*C-A*D);
  double xx, yy;

  if (param < 0)
  {
    xx = x0;
    yy = y0;
  }
  else if (param > 1)
  {
    xx = x1;
    yy = y1;
  }
  else
  {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  return sign0*distance(pX, pY, xx, yy);
}

using namespace std;
static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}

// angle diff from b to a
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
class ScanPoint
{
public:
  double x;
  double y;
};
class LaserData
{
public:
  LaserData(const sensor_msgs::LaserScan::ConstPtr& scan);
  ~LaserData();
  int mRangeCount;
  double mRangeMax;
  double (*mRanges)[2];
  std::vector<double> mRange;
  std::vector<int> nRange;
  std::vector<ScanPoint> m;
  
  double angle_min;
  double angle_inc;
};
LaserData::LaserData(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  mRangeCount = scan->ranges.size();
  mRanges =  new double[mRangeCount][2];
  mRangeMax = scan->range_max;
  double x=0.0,y=0.0;
  angle_min=scan->angle_min;
  angle_inc=scan->angle_increment;
  angle_inc=fmod(angle_inc + 5*M_PI,2*M_PI)-M_PI;
  for(int i=0;i<mRangeCount;i++)
  {
    if(scan->ranges[i]<=scan->range_min)
    {
      mRanges[i][0]=scan->range_max;
    }
    else 
    {
      mRanges[i][0]=scan->ranges[i];
    }
   
    mRanges[i][1] = angle_min + (i*angle_inc);
    x=mRanges[i][0]*sin(mRanges[i][1]);
    y=mRanges[i][0]*cos(mRanges[i][1]);
    ScanPoint sp;
    sp.x=x;
    sp.y=y;
    m.push_back(sp);
    mRange.push_back(mRanges[i][0]);
    nRange.push_back(i);
    
  }
  
}
LaserData::~LaserData()
{
  delete[] mRanges;
}
class Area
{
public:
  Area(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool dccarea;
  bool stoparea;
  int count;
  double r;
  double a_up;
  double b_down;
  void free(const std::vector<ScanPoint>& mm);
};
//前方是y轴正方向，左是x轴正方向
Area::Area(const sensor_msgs::LaserScan::ConstPtr& scan):
  stoparea(false),
  dccarea(false),
  r(0.4),
  a_up(1.0),
  b_down(0.9),
  count(0)
{
  LaserData ld(scan);
  count = ld.mRangeCount;
  free(ld.m);
  
}
void Area::free(const std::vector<ScanPoint>& mm)
{
  for(int i=0;i<mm.size();i++)
  {
    if(i>count/4&&i<count*3/4)
    {
      continue;
    }
    //ROS_ERROR("X:%f,Y:%f,i:%d",mm[i].x,mm[i].y,i);
    if(mm[i].x<r&&mm[i].x>-1*r)
    {
      if(mm[i].y<b_down)
      {
        stoparea= true;
      }
    }

    
  }
}
class CLC
{
public:
  CLC();
  ~CLC();
  bool init();

private:

/********
模式：去到目标点或循环
*********/
/**************
状态：包括空闲，开始，执行，完成
*************/
  enum { IDLE=0,
         START,
         ACTIVE,
         COMPLETED       
       } state_;

  std::string robot_frame_;
  std::string world_frame_;

//采用字典存储路径
  std::map<std::string,std::map<std::string,geometry_msgs::Pose> > traj_;
  std::map<std::string,geometry_msgs::Pose>::iterator next_iter_;
  std::map<std::string,geometry_msgs::Pose>::iterator current_iter_;
  std::map<std::string,geometry_msgs::Pose> init_position;
  double vi,r,kv,vmax,vmin;
  bool re_traj1_;
  bool re_cmd_;
  bool re_odom_;
  bool re_traj_;
  bool re_data_;
  bool stop_;
  yocs_msgs::TrajectoryList traj_list_;
  yocs_msgs::NavigationControl command_;
  std::deque<yocs_msgs::NavigationControl> cmd_que_;
  nav_msgs::Odometry odom_;
  std_msgs::UInt64MultiArray marray_;
  tf::TransformListener tf_;
  ros::Publisher pub_vel_;
  ros::Publisher status_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber waypoints_sub_;
  ros::Subscriber nav_sub_;
  ros::Subscriber trajectories_sub_;
  ros::Subscriber laserSubscriber;
  ros::Subscriber pause_sub_;
  boost::thread* recCommand_thread_;
  boost::condition_variable recCommand_cond_;
  boost::mutex recCommand_mutex_;
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
  void navCallback(const yocs_msgs::NavigationControl::ConstPtr &msg);
  void dataCallback(const std_msgs::UInt64MultiArray::ConstPtr &msg)
  void trajectoriesCallback(const yocs_msgs::TrajectoryList::ConstPtr& trajs);

  void publishStatusUpdate(const uint8_t& status);
  void gotopoint();

  bool getdistancer2g(double &mdistance,double &mangle);
  void global2local(const double& gx,const double& gy,const double& cx,const double& cy,const double& ca,double& m_x,double& m_y);
  void receiveTrajs();
  bool receiveTrajsOne();
  bool recCommand();
  bool getrobotpose(double& cx,double& cy,double& ca);
  void velLine();
  void velLineback();
  void spinRoute();

};
CLC::CLC(): 
            state_(IDLE),
            re_cmd_(false),
            re_odom_(false),
            re_traj_(false),
            re_traj1_(false),
            stop_(false),
            re_data_(false)
{}
CLC::~CLC()
{

  recCommand_thread_->interruption_requested();
  recCommand_thread_->join();
  delete recCommand_thread_;
}
bool CLC::init()
{

  ros::NodeHandle nh;
  ros::NodeHandle robotNode("~");
  robotNode.param("vi",vi,0.2);
  robotNode.param("r",r,0.2);
  robotNode.param("kv",kv,0.3);
  robotNode.param("vmax",vmax,0.5);
  robotNode.param("vmin",vmin,0.3);
  robotNode.param("robot_frame",robot_frame_,std::string("/base_footprint"));
  robotNode.param("world_frame",world_frame_,std::string("/map"));
  laserSubscriber=nh.subscribe("/scan",1,&CLC::scanCallback,this);
  odom_sub_=nh.subscribe("odom",1,&CLC::odomCallback,this);
  waypoints_sub_=nh.subscribe("/trajectories",1,&CLC::trajectoriesCallback,this);
  nav_sub_=nh.subscribe("/nav_ctrl",1,&CLC::navCallback,this);
  pause_sub_=nh.subscribe("/data_vector",5,&FoxconnReciever::dataCallback,this);
  pub_vel_=nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1);
  
  status_pub_ = nh.advertise<yocs_msgs::NavigationControlStatus>("nav_ctrl_status",1,true);
  recCommand_thread_= new boost::thread(boost::bind(&CLC::spinRoute,this));
  ros::Rate r(10);
  std::map<std::string,geometry_msgs::Pose> init_position;
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    if(re_traj_&& re_odom_)
    {
        
        std::string initp="initpose";
        init_position.insert(std::make_pair(initp,odom_.pose.pose));
        current_iter_=init_position.begin();
        return true;
    }
    else
      continue;
  }
}
void CLC::dataCallback(const std_msgs::UInt64MultiArray::ConstPtr &msg)
{
  marray_=*msg;
//  dataAnalysis();
  re_data_=true;
}
void CLC::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  Area area(scan);
  if(area.stoparea)
  {
    ROS_ERROR("STOP");
    stop_=true;
  }
  else
  {
    stop_ =false;
    ROS_ERROR("RUN");
  }
}

void CLC::navCallback(const yocs_msgs::NavigationControl::ConstPtr &msg)//ok
{
    command_=*msg;
    cmd_que_.push_back(command_);
    re_cmd_=true;
    ROS_ERROR("nav received!");
}

void CLC::odomCallback(const nav_msgs::OdometryConstPtr &odom)//ok
{
    odom_=*odom;
    re_odom_=true;
}

void CLC::trajectoriesCallback(const yocs_msgs::TrajectoryList::ConstPtr& trajs)//ok
{
  traj_list_ = *trajs;
  re_traj_=true;
  re_traj1_=true;
}
bool CLC::receiveTrajsOne()//ok
{
  std::map<std::string,geometry_msgs::Pose> _waypoints;
  unsigned int k=0,m=0;
  for(unsigned int traj=0;traj!=traj_list_.trajectories.size();traj++)
  {

    for(unsigned int wayp=0;wayp!=traj_list_.trajectories[traj].waypoints.size();wayp++)
    {

        _waypoints.insert(std::make_pair(traj_list_.trajectories[traj].waypoints[wayp].name,traj_list_.trajectories[traj].waypoints[wayp].pose));
      k++;
    }

    traj_.insert(std::make_pair(traj_list_.trajectories[traj].name,_waypoints));
    _waypoints.clear();
  }
  if(traj_list_.empty())
  {
    return false;
  }
  els
  {
    return true;
  }
/*
  std::map<string,std::map<std::string,geometry_msgs::Pose> >::iterator i=traj_.begin();
  
  while(i!=traj_.end())
  {
    ROS_ERROR("t:%s",i->first.c_str());
    std::map<string,geometry_msgs::Pose>::iterator j=i->second.begin();
    while(j!=i->second.end())
    {
      
      ROS_ERROR("w:%s",j->first.c_str());
      ++j;
    }
    ++i;
  }
*/
}

void CLC::receiveTrajs()//ok
{
  std::map<std::string,geometry_msgs::Pose> _waypoints;
  unsigned int k=0,m=0;
  for(unsigned int traj=0;traj!=traj_list_.trajectories.size();traj++)
  {

    for(unsigned int wayp=0;wayp!=traj_list_.trajectories[traj].waypoints.size();wayp++)
    {

        _waypoints.insert(std::make_pair(traj_list_.trajectories[traj].waypoints[wayp].name,traj_list_.trajectories[traj].waypoints[wayp].pose));
      k++;
    }

    traj_.insert(std::make_pair(traj_list_.trajectories[traj].name,_waypoints));
    _waypoints.clear();
  }
  

  std::map<string,std::map<std::string,geometry_msgs::Pose> >::iterator i=traj_.begin();
  
  while(i!=traj_.end())
  {
    ROS_ERROR("t:%s",i->first.c_str());
    std::map<string,geometry_msgs::Pose>::iterator j=i->second.begin();
    while(j!=i->second.end())
    {
      
      ROS_ERROR("w:%s",j->first.c_str());
      ++j;
    }
    ++i;
  }

}

void CLC::global2local(const double& gx,const double& gy,const double& cx,const double& cy,const double& ca,double& m_x,double& m_y)
{
  //将global中的物体转换到机器人坐标系
  m_x=gx*cos(ca)-gy*sin(ca)+cx;
  m_y=gx*sin(ca)+gy*cos(ca)+cy;

}



bool CLC::getrobotpose(double& cx,double& cy,double& ca)
{
  tf::Stamped<tf::Pose> global;
  tf::Stamped<tf::Pose> robot;
  global.setIdentity();
  robot.setIdentity();
  robot.frame_id_="base_footprint";
  robot.stamp_=ros::Time();
      try
      {
        tf_.waitForTransform("map","base_footprint",ros::Time(0),ros::Duration(3.0));
        tf_.transformPose("map",robot,global);
      }
      catch (tf::LookupException& ex)
      {
		ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
		return false;
      }
  cx=global.getOrigin().x();
  cy=global.getOrigin().y();
  ca=tf::getYaw(global.getRotation());
  return true;
}

bool CLC::getdistancer2g(double &mdistance,double &mangle)
{
  double cx=0;
  double cy=0;
  double ca=0;

  if(getrobotpose(cx,cy,ca))
  {}
  double a=atan2(next_iter_->second.position.y-cy,next_iter_->second.position.x-cx);
  mangle=angle_diff(a,ca);
  if(current_iter_==next_iter_)
  {}
  else
  { 
    mangle += distanceToLine(cx, cy,current_iter_->second.position.x,current_iter_->second.position.y, next_iter_->second.position.x, next_iter_->second.position.y);
  }
  mdistance=fabs(next_iter_->second.position.x-cx)+fabs(next_iter_->second.position.y-cy);
  return true;
}
void CLC::velLine()
{
  geometry_msgs::Twist cmd_vel;
  double cx=0;
  double cy=0;
  double ca=0;
  double alph=0.0;
  double dis=0.0;
  double r=0.0;
  if(getrobotpose(cx,cy,ca))
  {}
  else
  {ROS_ERROR("TF ERROR");}
  double error_dis=0.0;
  double la=atan2(next_iter_->second.position.y-current_iter_->second.position.y,next_iter_->second.position.x-current_iter_->second.position.x);
  if(getdistancer2g(error_dis,alph))
  {
//？
  }


  double mangle=angle_diff(la,ca);
  mangle=mangle+distanceToLine(cx, cy,current_iter_->second.position.x,current_iter_->second.position.y, next_iter_->second.position.x, next_iter_->second.position.y);
  double vel1=max(min(error_dis * kv, vmax), vmin);
  double acc_x = (vel1-odom_.twist.twist.linear.x)/10;
  double vel=odom_.twist.twist.linear.x+acc_x;
  if(vel/mangle<0.2 && vel/mangle>-0.2)
  {
    mangle=0.1;
    vel=0.0;
  }
  cmd_vel.angular.z=mangle;
  cmd_vel.linear.x=vel;
  pub_vel_.publish(cmd_vel);
}
void CLC::velLineback()
{
  geometry_msgs::Twist cmd_vel;
  double cx=0;
  double cy=0;
  double ca=0;
  double alph=0.0;
  double dis=0.0;
  double r=0.0;
  if(getrobotpose(cx,cy,ca))
  {}
  else
  {ROS_ERROR("TF ERROR");}
  double error_dis=0.0;
  double la=atan2(next_iter_->second.position.y-current_iter_->second.position.y,next_iter_->second.position.x-current_iter_->second.position.x);
        if(getdistancer2g(error_dis,alph))
        {
//？
        }

  double mangle=angle_diff(la,ca);
  mangle=mangle+distanceToLine(cx, cy,current_iter_->second.position.x,current_iter_->second.position.y, next_iter_->second.position.x, next_iter_->second.position.y);
  double vel1=max(min(error_dis * kv, vmax), vmin);
  double acc_x = (vel1-odom_.twist.twist.linear.x)/10;
  double vel=odom_.twist.twist.linear.x+acc_x;
  if(vel/mangle<0.2 && vel/mangle>-0.2)
  {
    mangle=0.1;
    vel=0.0;
  }
  cmd_vel.angular.z=-1*mangle;
  cmd_vel.linear.x=-1*vel;
  pub_vel_.publish(cmd_vel);

}
void CLC::publishStatusUpdate(const uint8_t& status)
{
  yocs_msgs::NavigationControlStatus msg;
  if (status == yocs_msgs::NavigationControlStatus::IDLING)
  {
    msg.status = yocs_msgs::NavigationControlStatus::IDLING;
    msg.status_desc = "Idling";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::RUNNING)
  {
    msg.status = yocs_msgs::NavigationControlStatus::RUNNING;
    msg.status_desc = "Navigating to way point.";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::PAUSED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::PAUSED;
    msg.status_desc = "Navigation on hold.";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::COMPLETED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::COMPLETED;
    msg.status_desc = "Reached final destination.";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::CANCELLED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::CANCELLED;
    msg.status_desc = "Navigation cancelled.";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::ERROR)
  {
    msg.status = yocs_msgs::NavigationControlStatus::ERROR;
    msg.status_desc = "An error occurred.";
    status_pub_.publish(msg);
  }
  else
  {
    ROS_ERROR_STREAM("Cannot publish unknown status updated!");
  }
}
void CLC::spinRoute()
{
  std::string st;
  ros::NodeHandle n;
  bool _init=true;
  std::map<std::string,geometry_msgs::Pose>::iterator _point_iter;
  std::map<std::string,std::map<std::string,geometry_msgs::Pose> >::iterator _traj_iter;
  static double time0=0;
  while(n.ok())
  {
    if(re_traj1_)
    {
      re_traj1_=false;
      if(receiveTrajsOne())
      {
        re_traj2_=true; 
      } 
    }
    if(re_traj2_)
    {
      if(re_data_)
      {
        //which route
        _traj_iter=traj_.begin();
        while(_traj_iter!=traj_.end())
        {
          sprintf(st,"%c%c",'w',marray_.data[0]);
          _point_iter=_traj_iter->second.find(st);
          if(_point_iter==_traj_iter->second.end())
          {
            ++_traj_iter;
          }
          else
          {
            //找到则跳出循环
            _init=true;
            foundroute=true;
            break;
          }
        }
        re_data_=false;
        
      }
    }
    //init next_iter_ and current_iter_
    if(_init)
    {
      current_iter_=_traj_iter->second.begin();
      next_iter_=current_iter_;
    }
    if(current_iter_==next_iter_)
    {
      if(next_iter_++==_traj_iter.end())
      {
        next_iter_=_traj_iter.begin();
      }
      else
      {
        next_iter_++;
      }
   
    }
    else
    {
        double cx=0;
        double cy=0;
        double ca=0;
        if(getrobotpose(cx,cy,ca))
        {}
        double mx=0;
        double my=0;
        double error_dis=0.0;
        double alph=0.0;
        if(getdistancer2g(error_dis,alph))
        {
//？
        }
        if(error_dis<0.2)
        {
          current_iter_=next_iter_;
          time0=ros::Time:now().toSec();
          for(int i=0;i!=marray.data.size();i++)
          {
            std::string st0;
            sprintf(st0,"%c%d",'w',i);
            if(marray.data[i]=='1')
            {
              if(current_iter_.first==st0)
              {
                while(ros::Time::now().toSec()-time0>15*60)
                {
                  ros::spin();
                }
                break;
              }
               
            }
          }
        }
        else
        {
          double gx=next_iter_->second.position.x;
          double gy=next_iter_->second.position.y;
          double mx=0,my=0;
          global2local(gx,gy,cx,cy,ca,mx,my);

          if(my>0)
            velLine();
          else if(my<0)
            velLineback();
        }
    }
  }
}

void CLC::recCommand()
{
  static bool foundroute=true;
  static bool reCMD=false;
  static bool route_completed=true;//判断有没有跑完，没跑完继续，跑完换路径
  std::map<std::string,std::map<std::string,geometry_msgs::Pose> >::iterator traj_iter_;//路径指针
  std::map<std::string,geometry_msgs::Pose>::iterator goal_iter_;//点指针
  static std::string goalName_;
  static std::string cmd_goal;
  static bool re_traj2_=false;
  static bool command_not_action=false;
  ROS_ERROR("FUCK");
  
  ros::NodeHandle n;
  while(n.ok())
  {
    if(re_traj1_)
    {
      re_traj1_=false;
      receiveTrajs();
      re_traj2_=true;
    }
    
    if((re_cmd_&&re_traj2_)||(route_completed&&command_not_action&&re_traj2_))
    {
//将命令存储等route_completed后继续执行
      //cmd_goal=command_.goal_name;
      cmd_goal=cmd_que_.front().goal_name;
      if(route_completed)
      {
        route_completed=false;
        command_not_action=false;
/*****************************plan***************************************/
        traj_iter_=traj_.begin();
        while(traj_iter_!=traj_.end())
        {
          goal_iter_ = traj_iter_->second.find(cmd_goal);
          if(goal_iter_ ==traj_iter_->second.end())
          {
            //如果没找到则换路径找
            ++traj_iter_;
          }
          else
          {
            next_iter_=traj_iter_->second.begin();
            goalName_=goal_iter_->first; 
            //找到则跳出循环
            foundroute=true;
            break;
          }

        }//while(traj_iter_!=traj_.end())

        if(!foundroute)
        {
          foundroute=false;
          ROS_ERROR("no point found!");
        }
      }//if(route_completed)
      else
      {
         command_not_action=true;
      }
/******************************************************************************/

      re_cmd_=false;
      reCMD=true;
    }// if(re_cmd_) 
/////////////////////////////////

      if(state_==IDLE)
      {
        publishStatusUpdate(yocs_msgs::NavigationControlStatus::IDLING);
        //if(command_.control == yocs_msgs::NavigationControl::START&&reCMD)
        if(cmd_que_.front().control == yocs_msgs::NavigationControl::START&&reCMD)
        {
          reCMD=false;
          state_=START;
        }
      }
      else if(state_==START)
      {
          publishStatusUpdate(yocs_msgs::NavigationControlStatus::RUNNING);
          state_=ACTIVE;
        
      }
      else if (state_ == ACTIVE)
      {
        //if(command_.control == yocs_msgs::NavigationControl::PAUSE||command_.control == yocs_msgs::NavigationControl::STOP)
        if(cmd_que_.front().control == yocs_msgs::NavigationControl::PAUSE||cmd_que_.front().control == yocs_msgs::NavigationControl::STOP)
        {
          state_ = IDLE;
        }
        
        double cx=0;
        double cy=0;
        double ca=0;
        if(getrobotpose(cx,cy,ca))
        {}
        double mx=0;
        double my=0;
        double error_dis=0.0;
        double alph=0.0;
        if(getdistancer2g(error_dis,alph))
        {
//？
        }
          if(error_dis<0.2)
          {
            current_iter_=next_iter_;//到目标点保存到上一个点
            if(next_iter_->first==goalName_)
            {
              state_ = COMPLETED;
              if(++next_iter_==traj_iter_->second.end()&&!route_completed)
              {
                ROS_ERROR("ROUTE COMPLETED");
                route_completed=true;
              }
            }
            else
            {
              next_iter_++;
              if(next_iter_==traj_iter_->second.end()&&!route_completed)
              {
                ROS_ERROR("ROUTE COMPLETED");
                route_completed=true;
                state_=COMPLETED;
              }
            }
            cmd_que_.pop_front();
          }

        if(stop_)
          continue;
        else
          velLine();
      }
      else if (state_ == COMPLETED)
      {
        publishStatusUpdate(yocs_msgs::NavigationControlStatus::COMPLETED); 
            
        //开始对接
        state_ = IDLE;
      }

  }//w
}
int main(int argc,char** argv)
{
  ros::init(argc,argv,"clc");
  CLC clc;
  
  if(clc.init())
  {
    ROS_ERROR("INIT");  
  }
  ros::spin();
  return 0;
}   
