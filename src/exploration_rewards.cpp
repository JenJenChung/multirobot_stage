#include "exploration_rewards.h"

void ExplorationRewards::mapCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const> msg,
                                     std::string &robot_name) {
  // report total explored m²
  uint32_t map_width = msg->info.width;
  uint32_t map_height = msg->info.height;

  float area = 0;

  for (uint32_t i = 0; i < map_width; ++i) {
    for (uint32_t j = 0; j < map_height; ++j) {
      if (msg->data[i * map_height + j] != -1) {
        area += 1;
      }
    }
  }

  area *= msg->info.resolution * msg->info.resolution;

  _explored_areas[robot_name] = area;

  // ROS_INFO("total area of %s: %f\n", robot_name.c_str(), area);

  // find min area of all robots which is the value used to evaluate performance of the policy
  if (_explored_areas.size() > 0) {
    // std::vector<float>::iterator min_area = std::min_element(_explored_areas.begin(), _explored_areas.end());

    auto min_area = std::max_element(_explored_areas.begin(), _explored_areas.end(),
                                     [](const std::pair<std::string, float> &p1,
                                        const std::pair<std::string, float> &p2) { return p1.second > p2.second; });

    std_msgs::Float64 area_msg;
    area_msg.data = min_area->second;
    _pub.publish(area_msg);
    // ROS_INFO("current min area is by %s: %f\n", min_area->first.c_str(), min_area->second);
  } else {
    ROS_INFO("_explored_areas still empty!\n");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rewards_node");
  ROS_INFO("rewards_node initialized");
  ros::NodeHandle nh("~");
  ExplorationRewards er(nh);
  ros::spin();
  return 0;
}
