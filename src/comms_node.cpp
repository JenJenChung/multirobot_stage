#include "comms_node.h"


void comms_node::mapInterceptCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const> msg,
             std::string& robot_name) {
    // intercept updates to robot_i/map and publish those to
    // robot_i/comms_node/robot_j/map whenever the robots are in range to communicate
    ROS_INFO("beginning of callback with robot name: %s!\n", robot_name.c_str());
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
        for (auto &r : _robot_names)
        {
            ROS_INFO("Looking at robot: %s\n", r.c_str());
            // if (true)
            if (r != robot_name)
            {
                auto other_robot_position = _robot_odoms[r]->pose.pose.position;
                float d = distance(callback_robot_position, other_robot_position);
                ROS_INFO("Distance from %s to %s: %f", robot_name.c_str(), r.c_str(), d);

                // always publish map from robot whose callback was called
                ROS_INFO("Publishing %s's map.\n", robot_name.c_str());
                std::string callback_robot_topic = robot_name + robot_name; // i since callback robot is j
                _map_pubs[callback_robot_topic]->publish(_robot_maps[robot_name]);
                // publish callback_robot and other_robot's maps such that they can be merged if close enough for communication
                if (d < comms_range)
                {
                    // TODO: problem: currently this publishes 3x for 2 robots, i.e. if it's callback 00,
                    // he publishes for 01, 10 and 11 when it should actually only publish for 01 or 10.
                    // publish other robot's map if nearby
                    if (_robot_maps.count(r) != 0) {
                        callback_robot_topic = robot_name + r; // 
                        ROS_INFO("And publishing %s's map.\n", r.c_str());
                        _map_pubs[callback_robot_topic]->publish(_robot_maps[r]);
                        // ROS_INFO("And done publishing %s's map.\n", r.first.c_str());
                    }

                }
            }
        }


        // make sure this is the topic the map merge is listening to
    }
    else
    {
        ROS_INFO("Key %s not found!\n", robot_name.c_str());
    }
    ROS_INFO("End of callback!\n");
}

// store most recent odom for each robot
void comms_node::OdometryInterceptCallback(const boost::shared_ptr<nav_msgs::Odometry const> msg,
                               std::string &robot_name)
{
    // _robot_odoms.insert_or_assign(robot_name, msg);  // C++17 only
    // ROS_INFO("Odom Callback begin with robot name: %s!\n", robot_name.c_str());
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