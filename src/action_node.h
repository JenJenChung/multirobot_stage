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
# include <nav_msgs/OccupancyGrid.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/Pose2D.h>
# include <geometry_msgs/Twist.h>
# include <geometry_msgs/TransformStamped.h>
# include <actionlib_msgs/GoalStatusArray.h>
# include <tf2_ros/transform_listener.h>
# include <tf2_geometry_msgs/tf2_geometry_msgs.h>
# include <Eigen/Dense>
# include <conversions.h>
# include <multirobot_stage/NeuralNet.h>

class ActionNode{
    public:
        ActionNode(ros::NodeHandle);
        ~ActionNode() {}
    
    private:
        ros::Subscriber map_sub_;
        ros::Subscriber *odom_sub_;
        ros::Subscriber stat_sub_;
        ros::Publisher action_pub_;
        ros::Publisher rec_map_pub_;

        int robot_id_;
        int n_robots_;
        int n_th_;
        ros::NodeHandle nh_;

        Eigen::MatrixXd current_state_;
        actionlib_msgs::GoalStatusArray current_status_;

        nav_msgs::OccupancyGrid *merged_map_;
        nav_msgs::Odometry *odoms_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_; 

        NeuralNet policy_;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&, nav_msgs::Odometry*);
        void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr&);

        void getWaypoint();
        Eigen::Vector3d getAction(const Eigen::MatrixXd&);
};

ActionNode::ActionNode(ros::NodeHandle n): tfListener_(tfBuffer_), policy_(0, 0, 0, LOGISTIC) {
    nh_ = n;

    // Initialize ROS params:
    nh_.param<int>("robot_id", robot_id_, 0);
    nh_.param<int>("/learning/nRob", n_robots_, 1);

    std::string rootns, merged_map_topic, odom_topic, action_topic, status_topic;
    nh_.param<std::string>("root_namespace", rootns, "robot");
    nh_.param<std::string>("merged_map_topic", merged_map_topic, "map");
    nh_.param<std::string>("odom_topic", odom_topic, "odom");
    nh_.param<std::string>("action_topic", action_topic, "action");
    nh_.param<std::string>("status_topic", status_topic, "move_base/status");
    
    // load policy network parameters
    int n_hidden;
    std::string aFun;
    nh_.param<int>("/learning/nBearings", n_th_, 256);
    nh_.param<int>("/learning/nHidden", n_hidden, 256);
    nh_.param<std::string>("/learning/actFun", aFun, "logistic");
    std::vector<double> AA, BB;
    std::stringstream A_param, B_param;
    A_param << "/" << rootns << "_" << robot_id_ << "/A";
    B_param << "/" << rootns << "_" << robot_id_ << "/B";
    nh_.getParam(A_param.str(), AA);
    nh_.getParam(B_param.str(), BB);
    MatrixXd A(n_th_*(1+n_robots_), n_hidden);
    MatrixXd B(n_hidden+1, 3);
    int ii = 0;
    for (int i=0; i<A.rows(); i++){
        for (int j=0; j<A.cols(); j++){
            A(i,j) = AA[ii];
            ii++;
        }
    }
    ii = 0;
    for (int i=0; i<B.rows(); i++){
        for (int j=0; j<B.cols(); j++){
            B(i,j) = BB[ii];
            ii++;
        }
    }
    policy_ = NeuralNet(n_th_*(1+n_robots_), 3, n_hidden, LOGISTIC);
    policy_.SetWeights(A, B);
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Weight matrices set!");
    odoms_ = new nav_msgs::Odometry[n_robots_];
    merged_map_ = new nav_msgs::OccupancyGrid;
    odom_sub_ = new ros::Subscriber[n_robots_];

    current_state_ = Eigen::MatrixXd::Zero(n_th_,1+n_robots_);
    
    // Initialize Subscribers and Publishers
    std::stringstream mergedmaptopic;
    mergedmaptopic << rootns << "_" << robot_id_ << "/" << merged_map_topic;
    map_sub_ = nh_.subscribe(mergedmaptopic.str(), 10 , &ActionNode::mapCallback, this);
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << mergedmaptopic.str());

    for (int r = 0; r < n_robots_; r++){
        std::stringstream robotopic;
        robotopic << rootns << "_" << r << "/" << odom_topic;
        odom_sub_[r] = nh_.subscribe<nav_msgs::Odometry>(robotopic.str(), 10, boost::bind(&ActionNode::odomCallback, this, _1, &odoms_[r]));
        ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << robotopic.str());
    }

    std::stringstream statustopic;
    statustopic << rootns << "_" << robot_id_ << "/" << status_topic;
    stat_sub_ = nh_.subscribe(statustopic.str(), 10, &ActionNode::statusCallback, this);
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << statustopic.str());

    std::stringstream actiontopic;
    actiontopic << rootns << "_" << robot_id_ << "/" << action_topic;
    action_pub_ = nh_.advertise<geometry_msgs::Twist>(actiontopic.str(),10);
    rec_map_pub_= nh_.advertise<visualization_msgs::MarkerArray>("rec_map",10);
}
    
void ActionNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    *merged_map_ = *msg;
    current_state_ = mapToPolar(*merged_map_, odoms_, robot_id_, n_robots_, n_th_);
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Current state:\n"<< current_state_);
    visualization_msgs::MarkerArray rec_map = polarToMarkerArray(current_state_, odoms_[robot_id_]);
    rec_map_pub_.publish(rec_map);
}

void ActionNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_in, nav_msgs::Odometry* msg_out){
    std::string map_frame = merged_map_->header.frame_id;
    std::string odom_frame = msg_in->header.frame_id;
    if (map_frame[0]=='/'){map_frame.erase(0,1);}
    if (odom_frame[0]=='/'){odom_frame.erase(0,1);}
    try{
        geometry_msgs::TransformStamped odom_trans = tfBuffer_.lookupTransform(map_frame, odom_frame, ros::Time(0));
        tf2::doTransform(msg_in->pose.pose, msg_out->pose.pose, odom_trans);
        msg_out->header = msg_in->header;
        msg_out->header.frame_id = map_frame;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void ActionNode::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    current_status_ = *msg;
    if (current_status_.status_list.size() !=0){
        if (current_status_.status_list[0].status == 2 || //
            current_status_.status_list[0].status == 3 || // Waypoint reached
            current_status_.status_list[0].status == 4 || // Failed to find valid plan. Even after recovery behaviour
            current_status_.status_list[0].status == 5 || //
            current_status_.status_list[0].status == 8){  //
            ROS_INFO_STREAM("Robot " << robot_id_ << ": Ready. Getting next way point");
            getWaypoint();
        }
    } else {
        ROS_INFO_STREAM("Robot " << robot_id_ << ": No status action available. Starting exploration");
        getWaypoint();
    }
}

void ActionNode::getWaypoint(){
    // This method outputs an action belonging to a state according to the loaded policy
    Eigen::Vector3d action = ActionNode::getAction(current_state_);
    geometry_msgs::Twist waypoint = polarToTwist(action, odoms_[robot_id_]);
    action_pub_.publish<geometry_msgs::Twist>(waypoint);
}

Eigen::Vector3d ActionNode::getAction(const Eigen::MatrixXd &state){
    // This method outputs an action belonging to a state according to the loaded policy
    // Slice the state (remove the bearing column) and reshape the state into a vector
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> M(state.block(0,1,n_th_,1+n_robots_));
    Eigen::Map<VectorXd> nn_input(M.data(), M.size());

    // Evaluate the network with the state as input
    Eigen::Vector3d action = policy_.EvaluateNN(nn_input);

    // convert NN output to feasible action on the map
    int t_idx = round(action(0)*n_th_);
    action(0) = action(0) * 2 * M_PI; // direction to go to, convert to radians
    action(1) = action(1) * state(t_idx,2); // distance to travel into direction, scaled by distance to frontier in that direction
    action(2) = action(2) * 2 * M_PI; // new heading of the robot, convert to radians
    return action;
}