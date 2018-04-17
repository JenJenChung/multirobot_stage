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
        ros::Subscriber map_sub_;
        ros::Subscriber *odom_sub_;
        ros::Publisher  action_pub_;

        int robot_id_;
        int nRobots_;
        ros::NodeHandle nh_;

        //multirobot_msgs::State current_state_;
        Eigen::MatrixXd current_state_;

        nav_msgs::OccupancyGrid merged_map_;
        nav_msgs::Odometry *odoms_;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&, nav_msgs::Odometry*);

        geometry_msgs::Twist getAction();
};

ActionNode::ActionNode(ros::NodeHandle n){
    /* TODOs:
        - initialize parameters
        - initialize/load policy
        - initialize publishers
    */
    nh_ = n;

    // Initialize params:
    robot_id_ = 0; // nh_.getParam();

    std::string merged_map_topic = "map"; // = nh_.getParam();
    std::string odom_topic  = "odom";
    std::string rootns = "robot"; // = nh_.getParam();
    nRobots_ = 2; // = nh_.getParam();

    odoms_ = new nav_msgs::Odometry[nRobots_];
    odom_sub_ = new ros::Subscriber[nRobots_];

    // Initialize Subscribers and Publishers
    std::stringstream mergedmaptopic;
    mergedmaptopic << rootns << "_" << robot_id_ << "/" << merged_map_topic;
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(mergedmaptopic.str(), 10 , &ActionNode::mapCallback, this);
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << mergedmaptopic.str());

    for (int r = 0; r < nRobots_; r++){
        std::stringstream robotopic;
        robotopic << rootns << "_" << r << "/" << odom_topic;
        odom_sub_[r] = nh_.subscribe<nav_msgs::Odometry>(robotopic.str(), 10, boost::bind(&ActionNode::odomCallback, this, _1, &odoms_[r]));
        ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << robotopic.str());
    }
}
    

void ActionNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    merged_map_ = *msg;
}

void ActionNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_in, nav_msgs::Odometry *msg_out)
{
  *msg_out = *msg_in;
}

geometry_msgs::Twist ActionNode::getAction(){
}