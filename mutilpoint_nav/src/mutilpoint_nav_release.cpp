/****************************************************
      Date:2018-07-06
      Author:Peakzuo
      Decribe:MutilPOINT_NAV_RELEASE,NOT USE KEYBOARD
******************************************************/
#include "../include/Mutilpoint_nav/mutilpoint_nav_release.hpp"

/**
 * @brief constructor
**/
Mutil_Point::Mutil_Point():x(0),y(0),w(0),first_state(false),second_sate(false),
    filename("/home/leishen/catkin_zz/src/mutilpoint_nav/param/mutilpoint.yaml")
{
   //nh.getParam("filename", filename);
   sub_initpose=nh.subscribe("/initialpose",100,&Mutil_Point::Initposecallback,this);
   sub_pose=nh.subscribe("/amcl_pose",100,&Mutil_Point::posecallback,this);
   sub_key=nh.subscribe("/key_type",100,&Mutil_Point::set_point,this);

}

/**
 * @brief Write the position to yaml file
**/
void Mutil_Point::set_point(const std_msgs::Int16 &key)
{
    static int16_t data_old=0;
    static int i=0;

    if(key.data==-1){
       second_sate = true;
       return;
    }

   if(key.data!=data_old){
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

    std::ofstream fout(filename);
    fout << node;
    i++;
    ROS_INFO("%d",i);
   }

   data_old = key.data;

}

/**
 * @brief Get Init_pose(Mouse point in RVIZ，use "2D Pose Estimate")
**/
void Mutil_Point::Initposecallback(const geometry_msgs::PoseWithCovarianceStamped &initialPose)
{

     frame_id=initialPose.header.frame_id;
     x=initialPose.pose.pose.position.x;
     y=initialPose.pose.pose.position.y;
     w=initialPose.pose.pose.orientation.w;

     first_state=true;

     //ROS_INFO("frame_id=%s,x=%f,y=%f,w=%f",frame_id.c_str(),x,y,w);
}

/**
 * @brief Get AMCL_POSE
**/
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
    char state[number]={'a','b','c'};
    yocs_msgs::WaypointList wps;
    yocs_msgs::TrajectoryList trajs;


    ros::init(argc,argv,"Mutilpoint_nav");
    Mutil_Point mutil_nav;

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
       //read position from yaml file
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
