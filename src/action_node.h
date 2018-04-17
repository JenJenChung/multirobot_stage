/*
This is the action node of robot_ID

Subscribed topics:
- robot_ID/merged_map (OccupancyGrid)

for i={0,..,n}
    - robot_i/odom (Odometry)

Published topics:
- robot_ID/next_waypoint (geometry_msgs/Twist)
- robot_ID/polar_state (Array)

Services:
- check if waypoint reached succesfully

*/
# include <ros/ros.h>
# include <math.h>
# include <algorithm> 
// # include <std_msgs/Int8.h>
# include <nav_msgs/OccupancyGrid.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/Pose2D.h>
# include <geometry_msgs/Twist.h>
# include <Eigen/Dense>

class ActionNode{
    public:
        ActionNode(ros::NodeHandle);
        ~ActionNode() {}
    
    private:
        ros::Subscriber map_sub;
        ros::Subscriber odom_sub[];
        ros::Publisher  action_pub;

        nav_msgs::OccupancyGrid merged_map_;
        nav_msgs::Odometry odoms_[];

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&);

        geometry_msgs::Twist getAction();
};

ActionNode::ActionNode(ros::NodeHandle n){
    /* TODOs:
        - initialize parameters
        - initialize/load policy
        - initialize subscribers and publishers
    */
}

void ActionNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&){
}

void ActionNode::odomCallback(const nav_msgs::Odometry::ConstPtr&){
}

geometry_msgs::Twist ActionNode::getAction(){
}