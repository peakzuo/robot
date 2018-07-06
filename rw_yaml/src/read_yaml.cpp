#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <yocs_msgs/Trajectory.h>
#include <yocs_msgs/Waypoint.h>

void test()
{
 geometry_msgs::PoseWithCovarianceStamped pose;

  YAML::Node node,wp_node;

  wp_node[0]["name"]="zuozuo";
  wp_node[0]["frame_id"]="map";
  wp_node[0]["pose"]["position"]["x"]=1;
  wp_node[0]["pose"]["position"]["y"]=0;
  wp_node[0]["pose"]["position"]["z"]=0;
  wp_node[0]["pose"]["orientation"]["x"]=0;
  wp_node[0]["pose"]["orientation"]["y"]=0;
  wp_node[0]["pose"]["orientation"]["z"]=0;
  wp_node[0]["pose"]["orientation"]["w"]=1;

  wp_node[1]["name"]="zz";
  wp_node[1]["frame_id"]="odom";
  wp_node[1]["pose"]["position"]["x"]=1;
  wp_node[1]["pose"]["position"]["y"]=0;
  wp_node[1]["pose"]["position"]["z"]=0;
  wp_node[1]["pose"]["orientation"]["x"]=0;
  wp_node[1]["pose"]["orientation"]["y"]=0;
  wp_node[1]["pose"]["orientation"]["z"]=0;
  wp_node[1]["pose"]["orientation"]["w"]=1;

  node["waypoints"]=wp_node;
  std::ofstream fout( "/home/leishen/catkin_zz/src/rw_yaml/param/param.yaml" );
  fout << node;

}



void test5()
{
    class Vector3
    {
    public:
        float x, y, z;

        void encode( YAML::Node &node )
        {
            node.push_back( x );
            node.push_back( y );
            node.push_back( z );
        }

        bool decode( YAML::Node &node )
        {

            if ( !node.IsSequence() || node.size() != 3 )
                return false;

            x = node[0].as<float>();
            y = node[1].as<float>();
            z = node[2].as<float>();

            return true;
        }
    };

    Vector3 pos;
    pos.x = 100.0f; pos.y = -45.0f; pos.z = 50.0f;

    YAML::Node node;
    pos.encode( node );

    std::ofstream fout( "/home/leishen/catkin_zz/src/rw_yaml/param/param.yaml" );
    fout << node;

    Vector3 pos2;
    pos2.decode( node );
}

void test6()
{
    //name: John Smith
    //age: 37
    //spouse:
    //    name: Jane Smith
    //    age: 25
    //children:
    //    -   name: Jimmy Smith
    //        age: 15
    //    -   name: Jenny Smith
    //        age 12

    YAML::Node node;
    node["name"] = "John Smith";
    node["age"] = 37;

    YAML::Node node2;
    node2["name"] = "Jane Smith";
    node2["age"] = 25;

    YAML::Node node3;
    node3["name"] = "Jimmy Smith";
    node3["age"] = 15;
    YAML::Node node4;
    node4["name"] = "Jenny Smith";
    node4["age"] = 12;

    node["spouse"] = node2;
    node["children"].push_back( node3 );
    node["children"].push_back( node4 );

    //node["children"].push_back( "{name: Alex Smith, age: 14}" );
    //node["children"].push_back( "name: Alex Smith, age: 14" );
    //node["children"].push_back( YAML::Load( "{name: Alex Smith, age: 14}" ) );
    //YAML::Node node5 = YAML::Load( "{name: Alex Smith, age: 14}" );
    //node["children"].push_back( node5 );

    std::ofstream fout( "/home/leishen/catkin_zz/src/rw_yaml/param/param.yaml"  );
    fout << node;
}

void test7()
{
    YAML::Node node;  // starts out as null
    node["key"] = "value";  // it now is a map node
    node["seq"].push_back("first element");  // node["seq"] automatically becomes a sequence
    node["seq"].push_back("second element");

    node["mirror"] = node["seq"][0];  // this creates an alias
    node["seq"][0] = "1st element";  // this also changes node["mirror"]
    node["mirror"] = "element #1";  // and this changes node["seq"][0] - they're really the "same" node

    node["self"] = node;  // you can even create self-aliases
    node[node["mirror"]] = node["seq"];  // and strange loops <img draggable="false" class="emoji" alt="🙂" src="https://s.w.org/images/core/emoji/2.4/svg/1f642.svg">


    std::ofstream fout( "/home/leishen/catkin_zz/src/rw_yaml/param/param.yaml" );
    fout << node;
}



int main(int argc, char **argv)
{
   /*
    std::string fin = "/home/leishen/catkin_zz/src/rw_yaml/param/param.yaml";       //yaml文件所在的路径
    YAML::Node yamlConfig = YAML::LoadFile(fin);
    int int_param = yamlConfig["int_param"].as<int>();
    std::cout << "node size: " << yamlConfig.size() << std::endl;
    std::cout << yamlConfig["bool_param"].as<bool>() << "\n";
    yamlConfig["bool_param"] = !yamlConfig["bool_param"].as<bool>();
    yamlConfig["str_param"] = "test";
    std::ofstream file;
    file.open(fin);
    file.flush();
    file << yamlConfig;
    file.close();
   */
test();

    return 0;
}
