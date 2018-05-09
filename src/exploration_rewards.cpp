#include "exploration_rewards.h"


void ExplorationRewards::mapCallback(const nav_msgs::OccupancyGrid &msg)
{
    // report total explored m²
    uint32_t map_width = msg.info.width;
    uint32_t map_height = msg.info.height;

    float area = 0;

    for (uint32_t i = 0; i < map_width; ++i) {
        for (uint32_t j = 0; j < map_height; ++j) {
            if (msg.data[i*map_height + j] != -1) {
                area += 1;
            }
        }
    }
 
    area *= msg.info.resolution * msg.info.resolution;

    std_msgs::Float32 area_msg;
    area_msg.data = area;
    _pub.publish(area_msg);
    ROS_INFO("total area: %f\n", area);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_rewards");

    ExplorationRewards er;

    ros::spin();
    return 0;
}
