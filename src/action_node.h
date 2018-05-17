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
# include <multirobot_stage/conversions.h>
# include <multirobot_stage/NeuralNet.h>
# include <move_base_msgs/MoveBaseAction.h>
# include <actionlib/client/simple_action_client.h>
# include <boost/thread.hpp>

class ActionNode{
    public:
        ActionNode(ros::NodeHandle);
        ~ActionNode() {}

        void spin();
    
    private:
        ros::Subscriber map_sub_;
        ros::Subscriber *odom_sub_;
        ros::Subscriber stat_sub_;
        //ros::Publisher action_pub_;
        ros::Publisher rec_map_pub_;

        int robot_id_;
        int n_robots_;
        int n_th_;
        ros::NodeHandle nh_;

        Eigen::MatrixXd current_state_;
        std::shared_ptr<nav_msgs::OccupancyGrid> merged_map_;
        std::vector<std::shared_ptr<nav_msgs::Odometry>> odoms_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_; 

        NeuralNet policy_;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&, std::shared_ptr<nav_msgs::Odometry>);

        move_base_msgs::MoveBaseGoal getGoal();
        Eigen::Vector3d getAction(const Eigen::MatrixXd&);
        void actionThread();
        
};

ActionNode::ActionNode(ros::NodeHandle n): tfListener_(tfBuffer_), policy_(0, 0, 0, LOGISTIC) {
    nh_ = n;

    // Initialize ROS params:
    nh_.param<int>("robot_id", robot_id_, 0);
    nh_.param<int>("/learning/nRob", n_robots_, 1);

    std::string rootns, merged_map_topic, odom_topic;
    nh_.param<std::string>("root_namespace", rootns, "robot");
    nh_.param<std::string>("merged_map_topic", merged_map_topic, "map");
    nh_.param<std::string>("odom_topic", odom_topic, "odom");
    
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
    for (int r = 0; r < n_robots_; r++){
        odoms_.push_back(std::make_shared<nav_msgs::Odometry>());  // create one odometry var per robot
    }
    // odoms_ = new nav_msgs::Odometry[n_robots_];
    merged_map_ = std::make_shared<nav_msgs::OccupancyGrid>();
    odom_sub_ = new ros::Subscriber[n_robots_];

    current_state_ = Eigen::MatrixXd::Zero(n_th_,2+n_robots_);
    
    // Initialize Subscribers and Publishers
    std::stringstream mergedmaptopic;
    mergedmaptopic << "/" << rootns << "_" << robot_id_ << "/" << merged_map_topic;
    map_sub_ = nh_.subscribe(mergedmaptopic.str(), 10 , &ActionNode::mapCallback, this);
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << mergedmaptopic.str());

    for (int r = 0; r < n_robots_; r++){
        std::stringstream robotopic;
        robotopic << "/" << rootns << "_" << r << "/" << odom_topic;
        odom_sub_[r] = nh_.subscribe<nav_msgs::Odometry>(robotopic.str(), 10, boost::bind(&ActionNode::odomCallback, this, _1, odoms_[r]));
        ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << robotopic.str());
    }

    rec_map_pub_= nh_.advertise<visualization_msgs::MarkerArray>("rec_map",10);
}
    
void ActionNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    *merged_map_ = *msg;
    // ROS_INFO("[mapCallback] merged_map_->header.frame_id: %s\n", merged_map_->header.frame_id.c_str());
    current_state_ = mapToPolar(*merged_map_, odoms_, robot_id_, n_robots_, n_th_);
    // ROS_INFO_STREAM("Robot " << robot_id_ << ": Current state:\n" << current_state_);
    visualization_msgs::MarkerArray rec_map = polarToMarkerArray(current_state_, *(odoms_[robot_id_]));
    rec_map_pub_.publish(rec_map);
}

void ActionNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_in, std::shared_ptr<nav_msgs::Odometry> msg_out){
    // TODO: hack, remove
    std::string map_frame = merged_map_->header.frame_id;
    std::string odom_frame = msg_in->header.frame_id;

    // ROS_INFO("[odomCallback] odom_frame: %s\tmerged_map_->header.frame_id: %s\n", odom_frame.c_str(), merged_map_->header.frame_id.c_str());

    if (map_frame[0]=='/'){map_frame.erase(0,1);}
    if (odom_frame[0]=='/'){odom_frame.erase(0,1);}
    try{
      geometry_msgs::TransformStamped odom_trans = tfBuffer_.lookupTransform(map_frame, odom_frame, ros::Time(0));
      geometry_msgs::PoseStamped pose_in, pose_out;
      pose_in.header = msg_in->header;
      pose_in.pose = msg_in->pose.pose;
      tf2::doTransform(pose_in, pose_out, odom_trans);
      msg_out->pose.pose = pose_out.pose;
      msg_out->header = pose_out.header;
      ROS_INFO("pose_in: %f, %f, %f\tpose_out: %f, %f, %f\n", pose_in.pose.position.x,
               pose_in.pose.position.y, pose_in.pose.position.z,  msg_out->pose.pose.position.x,
                msg_out->pose.pose.position.y,  msg_out->pose.pose.position.z);
      // if (map_frame[0]!='/'){map_frame.insert(0,"/");}
      // msg_out->header.frame_id = map_frame;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
}

move_base_msgs::MoveBaseGoal ActionNode::getGoal(){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = odoms_[robot_id_]->header.frame_id;
    // ROS_INFO("Robot %d - goal.target_pose.header.frame_id: %s\n", robot_id_, goal.target_pose.header.frame_id.c_str());
    goal.target_pose.header.stamp = ros::Time::now() ;

    Eigen::Vector3d action = getAction(current_state_);
    goal.target_pose.pose = polarToPose(action, *(odoms_[robot_id_]));  // TODO: dereferecing occurring in correct order?

    geometry_msgs::Twist waypoint = polarToTwist(action, *(odoms_[robot_id_]));
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Converted waypoint (geometry_msgs/Twist):\n" << waypoint);
    return goal;
}

Eigen::Vector3d ActionNode::getAction(const Eigen::MatrixXd &state){
    // This method outputs an action belonging to a state according to the loaded policy
    // ROS_INFO("Starting getAction()\n");
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> M(state.block(0,1,n_th_,1+n_robots_));
    Eigen::Map<VectorXd> nn_input(M.data(), M.size());
    Eigen::Vector3d action;
    // ROS_INFO("M.size: (%ld, %ld) nn_input.size: (%ld, %ld)\n", M.rows(), M.cols(), nn_input.rows(), nn_input.cols());
    if (nn_input.maxCoeff()!=0){
        nn_input = nn_input/nn_input.maxCoeff(); // scale input
        action = policy_.EvaluateNN(nn_input); // Evaluate the network with the state as input
    } else {
        action << 0, 0, 0;
        ROS_WARN_STREAM("Robot " << robot_id_ << ": Invalid input to neural net. Performing 0 action");
    }
    // convert NN output to feasible action on the map
    int t_idx = round(action(0)*n_th_);
    action(0) = action(0) * 2 * M_PI; // direction to go to, convert to radians
    action(1) = action(1) * state(t_idx,2); // distance to travel into direction, scaled by distance to frontier in that direction
    action(2) = action(2) * 2 * M_PI; // new heading of the robot, convert to radians
    // ROS_INFO("Finished getAction()\n");
    return action;
}

void ActionNode::actionThread(){
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true) ;
    actionlib::SimpleClientGoalState *goal_state = nullptr; 
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO_STREAM("Robot " << robot_id_ << ": Waiting for the move_base action server to come up");
    }
    ros::Rate r(5);
    while (ros::ok){
        r.sleep();
        // ROS_INFO("[actionThread] merged_map header frame_id: (%s) and seq: (%d)\n", merged_map_->header.frame_id.c_str(), merged_map_->header.seq);
        // ROS_INFO("[actionThread] odom header frame id for robot %d: %s.\n", robot_id_, odoms_[robot_id_]->header.frame_id.c_str());
        if (merged_map_) {
            // ROS_INFO("Found merged map!\n");
            if (!merged_map_->header.frame_id.empty()){
                // ROS_INFO("Merged map not empty\n");
                if (goal_state != nullptr){
                    if (goal_state->isDone()){  //
                        ROS_INFO_STREAM("Robot " << robot_id_ << ": Ready. Getting next way point");
                        *goal_state = ac.sendGoalAndWait(getGoal());
                        if(*goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
                            ROS_INFO_STREAM("Robot " << robot_id_ << ": Waypoint reached.");
                        } else {
                            ROS_INFO_STREAM("Robot " << robot_id_ << ": The base failed to reach the waypoint.") ;
                        }
                    } else {
                        ROS_INFO("goal_state is not Done, doing nothing...\n");
                    }
                } else {
                    ROS_INFO_STREAM("Robot " << robot_id_ << ": No action status available. Starting exploration");
                    goal_state = new actionlib::SimpleClientGoalState(ac.sendGoalAndWait(getGoal()));
                }
                if(*goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO_STREAM("Robot " << robot_id_ << ": Waypoint reached.");
                } else {
                    ROS_INFO_STREAM("Robot " << robot_id_ << ": The base failed to reach the waypoint.") ;
                }
            } else {
                ROS_INFO_STREAM("Robot " << robot_id_ << ": No map/status/state available. Waiting...");
                ros::Duration(1.0).sleep();
            }
        }
    }
}

void ActionNode::spin(){
    boost::thread action_thread(&ActionNode::actionThread,this);
    ros::spin();
    action_thread.join();
}