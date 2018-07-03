#ifndef _YAML_PARSER_H
#define _YAML_PARSER_H

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/WaypointList.h>

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif



  bool loadWaypointsAndTrajectoriesFromYaml(const std::string& filename,
                                            yocs_msgs::WaypointList& wps,
                                            yocs_msgs::TrajectoryList& trajs);
  void getYamlNode(const std::string& filename, YAML::Node& node);
  void parseWaypoints(const YAML::Node& node, yocs_msgs::WaypointList& wps);
  void parseTrajectories(const YAML::Node& node, const yocs_msgs::WaypointList& wps, yocs_msgs::TrajectoryList& trajs);


#endif
