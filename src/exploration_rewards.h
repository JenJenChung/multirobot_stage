#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float64.h"
#include <ros/console.h>
#include <ros/ros.h>

// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <tf/transform_datatypes.h>

class ExplorationRewards {

public:
  ExplorationRewards(ros::NodeHandle nodeHandle) {
    _nHandle = nodeHandle;
    std::string map_topic;
    int nRob;
    _nHandle.param<std::string>("merged_map_topic", map_topic, "/map_merge/map");
    
    _nHandle.param<int>("nRob", nRob, 0);
    if (nRob == 0) {
      ROS_WARN("Number of robots nRob set to 0!\n");
    }

    for (uint8_t i = 0; i < nRob; ++i) {
      _robot_names.push_back("robot_" + std::to_string(i));
      
      std::shared_ptr<ros::Subscriber> temp_map_sub = std::make_shared<ros::Subscriber>();
      std::string full_map_merge_topic = "/robot_" + std::to_string(i) + "/" + map_topic;
      *temp_map_sub = _nHandle.subscribe<nav_msgs::OccupancyGrid>(
          full_map_merge_topic, 10,
          boost::bind(&ExplorationRewards::mapCallback, this, _1, _robot_names.back()));
      
      ROS_INFO("Reward node subscribed to %s\n", full_map_merge_topic.c_str());
      
      _sub.insert(std::make_pair(_robot_names.back(), temp_map_sub));
    }
    _pub = _nHandle.advertise<std_msgs::Float64>("/reward", 10, true);
  }
  ~ExplorationRewards() {}

  void mapCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const> msg, std::string &robot_name);

private:
  ros::NodeHandle _nHandle;
  std::map<std::string, std::shared_ptr<ros::Subscriber>> _sub;
  ros::Publisher _pub;

  std::map<std::string, float> _explored_areas;
  std::vector<std::string> _robot_names;
};
