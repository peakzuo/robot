/****************************************
      Date:2018-06-20
      Author:Peakzuo
      Decribe:MutilPOINT_NAV
****************************************/
#include "../include/Mutilpoint_nav/Mutilpoint.h"

//constructor
Mutil_Point::Mutil_Point():x(0),y(0),w(0),start_state(false),
    second_sate(false),key_file_descriptor_(0),
    filename("/home/leishen/catkin_zz/src/mutilpoint_nav/param/mutilpoint.yaml")
{
   //nh.getParam("filename", filename);
   tcgetattr(key_file_descriptor_, &original_terminal_state_); // get terminal properties
   sub_initpose=nh.subscribe("/initialpose",100,&Mutil_Point::Initposecallback,this);
   sub_pose=nh.subscribe("/amcl_pose",100,&Mutil_Point::posecallback,this);

}

//destructor
Mutil_Point::~Mutil_Point()
{
  tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
}

bool Mutil_Point::init()
{

    bool connected = false;
    ecl::MilliSleep millisleep;
    int count = 0;

    while (!connected)
    {
      if (count == 6)
      {
        connected = false;
        break;
      }
      else
      {
        ROS_WARN_STREAM("Mutil: Could not connect, trying again after 500ms...");
        try
        {
          millisleep(500);
        }
        catch (ecl::StandardException& e)
        {
          ROS_ERROR_STREAM("Waiting has been interrupted.");
          ROS_DEBUG_STREAM(e.what());
          return false;
        }
        ++count;
      }
    }
    if (!connected)
    {
      ROS_ERROR("KeyOp: Could not connect.");
      ROS_ERROR("KeyOp: Check remappings for enable/disable topics.");
    }
    else
    {
      ROS_INFO("Mutil: connected.");
    }

    // start keyboard input thread
    thread_.start(&Mutil_Point::keyboardInputLoop, *this);
    return true;
}

void Mutil_Point::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state_, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor_, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Forward/back arrows : linear velocity incr/decr.");
  puts("Right/left arrows : angular velocity incr/decr.");
  puts("Spacebar : reset linear/angular velocities.");
  puts("d : disable motors.");
  puts("e : enable motors.");
  puts("q : quit.");
  char c;
  bool quit_requested_=false;
  while (!quit_requested_)
  {
    if (read(key_file_descriptor_, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }

    if(c=='b')
        set_point();
    else if(c=='v')
    {
        thread_.cancel();
        second_sate=true;
    }
  }
}


void Mutil_Point::set_point()
{
    static int i=0;

    wp_node[i]["name"]=i;
    wp_node[i]["frame_id"] = Get_pose.header.frame_id;
    wp_node[i]["pose"]["position"]["x"]=Get_pose.pose.pose.position.x;
    wp_node[i]["pose"]["position"]["y"]=Get_pose.pose.pose.position.y;
    wp_node[i]["pose"]["position"]["z"]=Get_pose.pose.pose.position.z;
    wp_node[i]["pose"]["orientation"]["x"]=Get_pose.pose.pose.orientation.x;
    wp_node[i]["pose"]["orientation"]["y"]=Get_pose.pose.pose.orientation.y;
    wp_node[i]["pose"]["orientation"]["z"]=Get_pose.pose.pose.orientation.z;
    wp_node[i]["pose"]["orientation"]["w"]=Get_pose.pose.pose.orientation.w;

    node["waypoints"]=wp_node;

    std::ofstream fout("/home/leishen/catkin_zz/src/mutilpoint_nav/param/mutilpoint.yaml");
    fout << node;
    i++;
    ROS_INFO("%d",i);

}


void Mutil_Point::Initposecallback(const geometry_msgs::PoseWithCovarianceStamped &initialPose)
{

     frame_id=initialPose.header.frame_id;
     x=initialPose.pose.pose.position.x;
     y=initialPose.pose.pose.position.y;
     w=initialPose.pose.pose.orientation.w;

     start_state=true;

     ROS_INFO("frame_id=%s,x=%f,y=%f,w=%f",frame_id.c_str(),x,y,w);
}


void Mutil_Point::posecallback(const geometry_msgs::PoseWithCovarianceStamped &Now_pose)
{
  Get_pose.header.frame_id=Now_pose.header.frame_id;
  Get_pose.pose.pose.position.x= Now_pose.pose.pose.position.x;
  Get_pose.pose.pose.position.y= Now_pose.pose.pose.position.y;
  Get_pose.pose.pose.orientation.x= Now_pose.pose.pose.orientation.x;
  Get_pose.pose.pose.orientation.y= Now_pose.pose.pose.orientation.y;
  Get_pose.pose.pose.orientation.z= Now_pose.pose.pose.orientation.z;
  Get_pose.pose.pose.orientation.w= Now_pose.pose.pose.orientation.w;

  ROS_INFO("x=%f,y=%f,o_x=%f,o_y=%f,o_z=%f,o_w=%f",Get_pose.pose.pose.position.x,Get_pose.pose.pose.position.y,
                   Get_pose.pose.pose.orientation.x,Get_pose.pose.pose.orientation.y, Get_pose.pose.pose.orientation.z,Get_pose.pose.pose.orientation.w);
}


int main(int argc,char **argv)
{
    int count;
    bool start_sign=false;
    std::string file_name;
    double x[number]={8.7,14.3,-3.5,10};
    double y[number]={2.56,-0.8,-3.6,11};
    double w[number]={1,1,1,1};
    char state[number]={'a','b','c'};
    yocs_msgs::WaypointList wps;
    yocs_msgs::TrajectoryList trajs;


    ros::init(argc,argv,"Mutilpoint_nav");
    Mutil_Point mutil_nav;
    mutil_nav.init();

    mutil_nav.Get_file_name(file_name);

    MoveBaseclient ac("move_base",true);


    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    ROS_INFO("Click on the map in Rviz to set the intial pose...");


    ros::Rate loop_rate(100);
    while(ros::ok())
    {
     mutil_nav.Get_startstate(start_sign);
     if(start_sign==true){
      if(!loadWaypointsAndTrajectoriesFromYaml(file_name, wps, trajs))
         {
            ROS_ERROR("Waypoint Provider : Failed to parse yaml[%s]",file_name.c_str());
           return -1;
         }

      for(count=0;count<wps.waypoints.size();count++)
      {
          goal.target_pose.header.frame_id="map";
          goal.target_pose.header.stamp=ros::Time::now();
          goal.target_pose.pose.position.x=wps.waypoints[count].pose.position.x;
          goal.target_pose.pose.position.y=wps.waypoints[count].pose.position.y;
          goal.target_pose.pose.orientation.w=wps.waypoints[count].pose.orientation.w;

          //ROS_INFO("%f,%f,%d",x[count],y[count],count);
          ROS_INFO("Sending goal");
          ac.sendGoal(goal);

          ac.waitForResult();

          if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
              ROS_INFO("%c,the base arrive the goal",state[count]);
          else
              ROS_INFO("%c,the base failed to arrive the goal",state[count]);

      }


     }
      ros::spinOnce();
      loop_rate.sleep();
     }

     return 0;
}
