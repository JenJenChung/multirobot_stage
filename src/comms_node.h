#include <algorithm>
#include <regex>
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

class comms_node
{
  public:
    comms_node()
    {

        // get number of active robots
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        std::vector<const char *> topic_strings;

        // TODO(bhahn): count number of robots (e.g. via published topics)
        // for (auto& topic : master_topics) {
        //     ROS_INFO("%s\n", topic.name.c_str());
        //     topic_strings.push_back(topic.name.c_str());
        // }

        // std::regex robot_topic_names("robot_\d/map", std::regex_constants::grep);
        // uint8_t num_robots = std::count_if(master_topics.begin(), master_topics.end(),
        //         std::regex_match(topic_strings, robot_topic_names));

        _nh.param<float>("comms_range", comms_range, 5.0);

        uint8_t num_robots = 2;
        ROS_INFO("num_robots: %d\n", num_robots);

        // subscribe to each robots odom, map topics
        // TODO(bhahn): for each robot i, publish other robot j's map in ns robot_i/comms_node/
        for (uint8_t i = 0; i < num_robots; ++i) {
          for (uint8_t j = 0; j < num_robots; ++j) {
            std::string robot_id, other_robot_id, robot_map_topic, robot_pub_topic_key, robot_odom_topic,
                robot_odom_topic_key, robot_map_topic_republished, robot_odom_topic_republished;
            robot_id = "robot_" + std::to_string(j);
            other_robot_id = "robot_" + std::to_string(i);
            robot_pub_topic_key = robot_id + other_robot_id;
            robot_map_topic = "robot_" + std::to_string(j) + "/map";
            robot_map_topic_republished =
                "robot_" + std::to_string(j) + "/comms_node/robot_" + std::to_string(i) + "/map";
            robot_odom_topic = "robot_" + std::to_string(j) + "/odom";
            robot_odom_topic_republished =
                "robot_" + std::to_string(j) + "/comms_node/robot_" + std::to_string(i) + "/odom";
            
            if (i != j) {
              std::shared_ptr<ros::Subscriber> temp_map_sub = std::make_shared<ros::Subscriber>();
              std::shared_ptr<ros::Subscriber> temp_odom_sub = std::make_shared<ros::Subscriber>();
              // robot_odom_topic_key = "robot_" + std::to_string(i) + std::to_string(j);
              ROS_INFO("robot_map_topic: %s\n", robot_map_topic.c_str());

              // temp_sub = _nh.subscribe(robot_map_topic, 10, boost::bind(&comms_node::mapInterceptCallback);
              _robot_names.push_back(robot_id);
              *temp_map_sub = _nh.subscribe<nav_msgs::OccupancyGrid>(
                  robot_map_topic, 10, boost::bind(&comms_node::mapInterceptCallback, this, _1, _robot_names.back()));
              _map_subs.insert(std::make_pair(robot_id, temp_map_sub));

              // TODO(bhahn): currently subscribing to odom 4 times, only need 2 but then have to fix the issue that
              // we'd use a different key. same with map_subs. Actually only need 4 pubs, right? And 4 odom pubs.
              //
              *temp_odom_sub = _nh.subscribe<nav_msgs::Odometry>(
                  robot_odom_topic, 10,
                  boost::bind(&comms_node::OdometryInterceptCallback, this, _1, _robot_names.back()));
              _odom_subs.insert(std::make_pair(robot_id, temp_odom_sub));
            }

            ROS_INFO("Creating publisher for map %s with key %s\n", robot_map_topic_republished.c_str(),
                     robot_pub_topic_key.c_str());
            std::shared_ptr<ros::Publisher> temp_map_pub = std::make_shared<ros::Publisher>();
            *temp_map_pub = _nh.advertise<nav_msgs::OccupancyGrid>(robot_map_topic_republished, 10);
            _map_pubs.insert(std::make_pair(robot_pub_topic_key, temp_map_pub));

            // publish odom of each robot when in range so other robot can read it
            ROS_INFO("Creating publisher for odom %s with key %s\n", robot_odom_topic_republished.c_str(),
                     robot_pub_topic_key.c_str());
            std::shared_ptr<ros::Publisher> temp_odom_pub = std::make_shared<ros::Publisher>();
            *temp_odom_pub = _nh.advertise<nav_msgs::Odometry>(robot_odom_topic_republished, 10);
            _odom_pubs.insert(std::make_pair(robot_pub_topic_key, temp_odom_pub));
          }
        }
    }

    ~comms_node() = default;

    void mapInterceptCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const> msg, std::string &robot_name);
    void OdometryInterceptCallback(const boost::shared_ptr<nav_msgs::Odometry const> msg, std::string &robot_name);

    float distance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
      float x2 = (p1.x - p2.x) * (p1.x - p2.x);
      float y2 = (p1.y - p2.y) * (p1.y - p2.y);
      float z2 = (p1.z - p2.z) * (p1.z - p2.z);
      return sqrt(x2 + y2 + z2);
    }

  private:
    ros::NodeHandle _nh;

    std::map<std::string, std::shared_ptr<ros::Subscriber>> _map_subs;
    std::map<std::string, std::shared_ptr<ros::Subscriber>> _odom_subs;
    std::map<std::string, std::shared_ptr<ros::Publisher>> _map_pubs;
    std::map<std::string, std::shared_ptr<ros::Publisher>> _odom_pubs;

    std::map<std::string, boost::shared_ptr<nav_msgs::OccupancyGrid const>> _robot_maps;
    std::map<std::string, boost::shared_ptr<nav_msgs::Odometry const>> _robot_odoms;

    std::vector<std::string> _robot_names;

    float comms_range = 5.0;
};