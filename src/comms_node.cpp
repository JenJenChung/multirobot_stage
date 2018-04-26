#include "comms_node.h"


void comms_node::mapInterceptCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const> msg,
             std::string& robot_name) {
    // intercept updates to robot_i/map and publish those to
    // robot_i/map_in_range whenever the robots are in range to communicate
    // ROS_INFO("beginning of callback!\n");
    float comms_range = 8.0;

    // store robot map
    _robot_maps[robot_name] = msg;

    if (_robot_odoms.count(robot_name) != 0) {
        // ROS_INFO("mapInterceptCallback by %s with position (%f)\n", 
        //             robot_name.c_str(), _robot_odoms[robot_name]->twist.twist.linear.x);
        // ROS_INFO("mapInterceptCallback by %s with position (%f, %f, %f)\n", 
        //             robot_name.c_str(), _robot_odoms[robot_name]->twist.twist.linear.x,
        //             _robot_odoms[robot_name]->twist.twist.linear.y, _robot_odoms[robot_name]->twist.twist.linear.z);

        // get odometry, get relative distance.
        // Limitation: this is only for two robots; since all robots will have access to merged map
        auto callback_robot_position = _robot_odoms[robot_name]->pose.pose.position;

        // calculate distance to all other robots
        // TODO: currently only works for n=2 robots
        for (auto &r : _robot_odoms)
        {
            if (r.first != robot_name)
            {
                auto other_robot_position = r.second->pose.pose.position;
                float d = distance(callback_robot_position, other_robot_position);
                ROS_INFO("Distance from %s to %s: %f", robot_name.c_str(), r.first.c_str(), d);

                // publish callback_robot and other_robot's maps such that they can be merged if close enough for communication
                if (d < comms_range) {
                    ROS_INFO("Publishing %s's map.\n", robot_name.c_str());
                    _map_pubs[robot_name]->publish(_robot_maps[robot_name]);
                    ROS_INFO("Done publishing %s's map.\n", robot_name.c_str());
                    if (_robot_maps.count(r.first) != 0) {
                        ROS_INFO("And publishing %s's map.\n", r.first.c_str());
                        _map_pubs[r.first]->publish(_robot_maps[r.first]);
                        ROS_INFO("And done publishing %s's map.\n", r.first.c_str());
                    }

                }
            }
        }


        // make sure this is the topic the map merge is listening to
    }
    else
    {
        ROS_INFO("Key not found!\n");
    }
}

// store most recent odom for each robot
void comms_node::OdometryInterceptCallback(const boost::shared_ptr<nav_msgs::Odometry const> msg,
                               std::string &robot_name)
{
    // _robot_odoms.insert_or_assign(robot_name, msg);  // C++17 only
    _robot_odoms[robot_name] = msg;
    // ROS_INFO("twist.x: %f\n", _robot_odoms[robot_name]->twist.twist.linear.x);
    // ROS_INFO("OdometryInterceptCallback by %s\n", robot_name.c_str()); 
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "comms_node");

    comms_node cn;
    ROS_INFO("comms_node initialized");

    ros::spin();

    return 0;
}


// TODO: hack gmapping and subscribe to global map merge and replace whenever a new map is merged
//       to maintain only one map. 