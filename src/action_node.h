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

Logic:
    - subscribe to local shared map
    - subscribe to odometries
    - while not finished:
        - calculate current_state_ according to map and last known odometries
        - if ready_flag:
            - getAction(current_state_)
            - ready_flag=False # the ready flag will be set to True if the robot has reached the waypoint, conducted measurements and exchanged information
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
# include <conversions.h>

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
        geometry_msgs::Twist getAction(const Eigen::MatrixXd&);
};

ActionNode::ActionNode(ros::NodeHandle n){
    /* TODOs:
        - initialize parameters
        - initialize/load policy
        - initialize publishers
    */
    nh_ = n;

    // Initialize ROS params:
    nh_.param<int>("robot_id", robot_id_, 0);
    nh_.param<int>("nRobots", nRobots_, 1);

    std::string rootns, merged_map_topic, odom_topic, action_topic; 
    nh_.param<std::string>("root_namespace", rootns, "robot");
    nh_.param<std::string>("merged_map_topic", merged_map_topic, "map");
    nh_.param<std::string>("odom_topic", odom_topic, "odom");
    nh_.param<std::string>("action_topic", action_topic, "action");

    odoms_ = new nav_msgs::Odometry[nRobots_];
    odom_sub_ = new ros::Subscriber[nRobots_];

    // Load policy
    /* load the policy, possible formats:
        - weights or other parameters of a predefined function
        - a generic function defined in a separate file or script:
    */

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
    std::stringstream actiontopic;
    actiontopic << rootns << "_" << robot_id_ << "/" << action_topic;
    action_pub_ = nh_.advertise<geometry_msgs::Twist>(actiontopic.str(),10);
}
    
void ActionNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    merged_map_ = *msg;
    // TODO: this callback will also calculate the current_state
}

void ActionNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_in, nav_msgs::Odometry *msg_out)
{
    *msg_out = *msg_in;
}

geometry_msgs::Twist ActionNode::getAction(){
    // This method outputs an action belonging to a state according to the loaded policy
    geometry_msgs::Twist action = ActionNode::getAction(current_state_);
    action_pub_.publish<geometry_msgs::Twist>(action);
    return action;
}

geometry_msgs::Twist ActionNode::getAction(const Eigen::MatrixXd &state){
    // This method outputs an action belonging to a state according to the loaded policy
    geometry_msgs::Twist action; 
    // TODO: insert policy here
    return action;
}