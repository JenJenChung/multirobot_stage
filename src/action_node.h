/*
This is the action node of robot_ID

Subscribed topics:
- robot_ID/comms_node/robot_ID/merged_map (OccupancyGrid)

for i={0,..,n}
    - robot_ID/comms_node/robot_i/odom (Odometry)

Published topics:
- robot_ID/rec_map (VisualizationMarkerArray)

Logic:
    - subscribe to local merged map
    - subscribe to odometries
    - while not finished:
        - calculate current_state_ according to map and last known odometries
        - if isDone():
            - record reward and save and update policy (if learning)
            - getAction(current_state_)
*/

# include <ros/ros.h>
# include <math.h>
# include <algorithm> 
# include <nav_msgs/OccupancyGrid.h>
# include <nav_msgs/Odometry.h>
# include <std_msgs/Float64.h>
# include <geometry_msgs/Pose2D.h>
# include <geometry_msgs/Twist.h>
# include <geometry_msgs/TransformStamped.h>
# include <actionlib_msgs/GoalStatusArray.h>
# include <tf2_ros/transform_listener.h>
# include <tf2_geometry_msgs/tf2_geometry_msgs.h>
# include <Eigen/Dense>
# include <multirobot_stage/conversions.h>
# include <multirobot_stage/PolicyGrad.h>
# include <move_base_msgs/MoveBaseAction.h>
# include <actionlib/client/simple_action_client.h>
# include <boost/thread.hpp>
# include <random>

enum runMode {LEARNING, EXECUTING}; 
enum runMeth {NE, PG, PGA}; 

class ActionNode{
    public:
        ActionNode(ros::NodeHandle);
        ~ActionNode() {}

        void spin();
    
    private:
        ros::Subscriber map_sub_;
        ros::Subscriber *odom_sub_;
        ros::Subscriber rew_sub_;
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

        PolicyGrad *policy_;
        runMode mode_;
        runMeth method_;

        double current_reward_;
        double last_reward_;
        double clearance_;

        std::default_random_engine generator_;

        std::string log_dir;
        std::string action_log_file_name;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &, std::shared_ptr<nav_msgs::Odometry>);
        void rewardCallback(const std_msgs::Float64::ConstPtr &);

        void updatePolicy(Eigen::MatrixXd &, int &);
        void savePolicy();
        move_base_msgs::MoveBaseGoal getGoal(Eigen::MatrixXd &, int &);
        void getAction(const Eigen::MatrixXd, int&);
        void actionThread();
        
};

ActionNode::ActionNode(ros::NodeHandle n): tfListener_(tfBuffer_) {
    nh_ = n;

    // Initialize ROS params:
    nh_.param<int>("robot_id", robot_id_, 0);
    nh_.param<int>("/learning/nRob", n_robots_, 1);

    std::string rootns, merged_map_topic, odom_topic, reward_topic;;
    nh_.param<std::string>("root_namespace", rootns, "robot");
    nh_.param<std::string>("merged_map_topic", merged_map_topic, "map");
    nh_.param<std::string>("odom_topic", odom_topic, "odom");
    nh_.param<std::string>("reward_topic", reward_topic, "/reward");
    nh_.param<double>("frontier_clearance", clearance_, 1);
    
    // load policy network and learning parameters
    int n_hidden;
    std::string aFun, mode, method;
    nh_.param<int>("/learning/nBearings", n_th_, 64);
    nh_.param<int>("/learning/nHidden", n_hidden, 256);
    nh_.param<std::string>("/learning/actFun", aFun, "logistic");
    nh_.param<std::string>("/learning/method", method, "NE");
    nh_.param<std::string>("/learning/mode", mode, "learning");
    double rate, beta1, beta2, eps;
    nh_.param<double>("/learning/rate", rate, 0.001);
    nh_.param<double>("/learning/beta1", beta1, 0.9);
    nh_.param<double>("/learning/beta2", beta2, 0.999);
    nh_.param<double>("/learning/eps", eps, 10e-8);
    
    ROS_INFO_STREAM("network" << n_th_*(1+n_robots_) << " " << n_th_ << " " << n_hidden);
    if (aFun=="tanh"){
        policy_ = new PolicyGrad(n_th_*(1+n_robots_), n_th_, n_hidden, TANH, rate, beta1, beta2, eps);
    } else {
        policy_ = new PolicyGrad(n_th_*(1+n_robots_), n_th_, n_hidden, LOGISTIC, rate, beta1, beta2, eps);
    }

    // load policy network weights
    std::vector<double> AA, BB;
    std::stringstream A_param, B_param;
    A_param << "/" << rootns << "_" << robot_id_ << "/A";
    B_param << "/" << rootns << "_" << robot_id_ << "/B";
    nh_.getParam(A_param.str(), AA);
    nh_.getParam(B_param.str(), BB);
    MatrixXd A(n_th_*(1+n_robots_), n_hidden);
    MatrixXd B(n_hidden+1, n_th_);
    if (AA.size()==0 || AA.size()==0){
        ROS_INFO_STREAM("Robot " << robot_id_ << ": Weight matrices randomly initialised!");
    } else {
        std::size_t ii = 0;
        for (std::size_t i=0; i<A.rows(); i++){
            for (std::size_t j=0; j<A.cols(); j++){
                A(i,j) = AA[ii];
                ii++;
            }
        }
        ii = 0;
        for (std::size_t i=0; i<B.rows(); i++){
            for (std::size_t j=0; j<B.cols(); j++){
                B(i,j) = BB[ii];
                ii++;
            }
        }
        policy_->SetWeights(A, B);
        ROS_INFO_STREAM("Robot " << robot_id_ << ": Weight matrices set by Rosparam!");
    }
    
    if (mode=="executing"){
        mode_ = EXECUTING;
    } else {
        mode_ = LEARNING;
    }

    if (method=="PG"){
        method_ = PG;
    } else if (method=="PGA"){
        method_ = PGA;
    } else {
        method_ = NE;
    }

    current_reward_ = 0;
    last_reward_ = 0;

    for (std::size_t r = 0; r < n_robots_; r++){
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

    if (n_robots_>1){
        for (std::size_t r = 0; r < n_robots_; r++){
            std::stringstream robotopic;
            robotopic << "/" << rootns << "_" << robot_id_ << "/comms_node/" << rootns << "_" << r << "/" << odom_topic;
            odom_sub_[r] = nh_.subscribe<nav_msgs::Odometry>(robotopic.str(), 10, boost::bind(&ActionNode::odomCallback, this, _1, odoms_[r]));
            ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << robotopic.str());
        }
    } else if (n_robots_==1){
        std::stringstream robotopic;
        robotopic << "/" << rootns << "_" << robot_id_ << "/" << odom_topic;
        odom_sub_[0] = nh_.subscribe<nav_msgs::Odometry>(robotopic.str(), 10, boost::bind(&ActionNode::odomCallback, this, _1, odoms_[0]));
        ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << robotopic.str());
    }

    rew_sub_ = nh_.subscribe(reward_topic, 10 , &ActionNode::rewardCallback, this);
    ROS_INFO_STREAM("Robot " << robot_id_ << ": Subscribed to: " << reward_topic);

    rec_map_pub_= nh_.advertise<visualization_msgs::MarkerArray>("rec_map",10);

    // initialize log file
    std::ofstream action_log_file;
    std::string curEpoch, curEpisode;
    ros::param::get("/learning/curEpoch", curEpoch);
    ros::param::get("/learning/curEpisode", curEpisode);
    ros::param::get("/learning/log_dir", log_dir);

    action_log_file_name = "action_log_robot_" + std::to_string(robot_id_) + '-' + curEpoch + "-" + curEpisode + ".csv";
    ROS_INFO("=================> ActionNode: writing file to %s\n", action_log_file_name.c_str());
    action_log_file.open(log_dir + "/" + action_log_file_name, std::ios_base::app);
    action_log_file << "time";
      action_log_file << "," + std::to_string(robot_id_) + "_action";
    action_log_file << std::endl;
}
    
void ActionNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    *merged_map_ = *msg;
}

void ActionNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_in, std::shared_ptr<nav_msgs::Odometry> msg_out){
    // TODO: hack, remove
    std::string map_frame = merged_map_->header.frame_id;
    std::string odom_frame = msg_in->header.frame_id;

    //ROS_INFO("[Robot-%i-odomCallback] odom_frame: %s\tmerged_map_->header.frame_id: %s\n", robot_id_, odom_frame.c_str(), merged_map_->header.frame_id.c_str());

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

        current_state_ = mapToPolar(*merged_map_, odoms_, robot_id_, n_th_);
        rec_map_pub_.publish(polarToMarkerArray(current_state_, *(odoms_[robot_id_])));

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void ActionNode::rewardCallback(const std_msgs::Float64::ConstPtr& msg){
    current_reward_ = msg->data;
}

void ActionNode::updatePolicy(Eigen::MatrixXd &last_state, int &last_action_index){
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> M(last_state.block(0,1,n_th_,1+n_robots_));
    Eigen::Map<VectorXd> nn_input(M.data(), M.size());
    if (nn_input.maxCoeff()!=0){
        nn_input = nn_input/nn_input.maxCoeff(); // scale input

        double reward = current_reward_ - last_reward_;
        if (last_action_index>-1 && last_reward_>0 && mode_==LEARNING){
            last_reward_ = current_reward_;
            if (method_==PGA){
                ROS_INFO_STREAM("Robot " << robot_id_ << ": Reward increment: " << reward << ". Updating policy with ADAM...");
                policy_->PolicyGradientADAMStep(nn_input, last_action_index, reward);
            } else if (method_==PG){
                ROS_INFO_STREAM("Robot " << robot_id_ << ": Reward increment: " << reward << ". Updating policy...");
                policy_->PolicyGradientStep(nn_input, last_action_index, reward);
            }
        } else {
            last_reward_ = current_reward_;
            ROS_WARN_STREAM("Robot " << robot_id_ << ": No previous action available for policy update.");
        }
    }
}

void ActionNode::savePolicy(){
    // Reshape weight matrices to single vector
    MatrixXd A = policy_->GetWeightsA() ;
    vector<double> AA ;
    for (int ii = 0; ii < A.rows(); ii++){
        for (int jj = 0; jj < A.cols(); jj++){
            AA.push_back(A(ii,jj)) ;
        }
    }
    MatrixXd B = policy_->GetWeightsB() ;
    vector<double> BB ;
    for (int ii = 0; ii < B.rows(); ii++){
        for (int jj = 0; jj < B.cols(); jj++){
            BB.push_back(B(ii,jj)) ;
        }
    }
    // Write NN policies to textfile
    char strA[50] ;
    char strB[50] ;
    sprintf(strA,"weightsA%d.txt",robot_id_) ;
    sprintf(strB,"weightsB%d.txt",robot_id_) ;
    policy_->OutputNN(strA,strB) ;

    // Write NN policies to rosparams
    sprintf(strA,"/robot_%d/A",robot_id_) ;
    sprintf(strB,"/robot_%d/B",robot_id_) ;
    ros::param::set(strA,AA) ;
    ros::param::set(strB,BB) ;
}

move_base_msgs::MoveBaseGoal ActionNode::getGoal(Eigen::MatrixXd &last_state, int &last_action_index){
    move_base_msgs::MoveBaseGoal goal;
    nav_msgs::Odometry current_odom = *(odoms_[robot_id_]);
    goal.target_pose.header = current_odom.header;
    if (current_odom.header.frame_id.empty()){
      goal.target_pose.header.frame_id = "/map";
      ROS_WARN_STREAM("Robot " << robot_id_ << ": Empty frame received. Using default frame /map for goal.");
    }
    int action_index = -1;
    Eigen::MatrixXd state = current_state_;
    getAction(state, action_index);
    last_state = state;
    last_action_index = action_index;
    Eigen::Vector3d action;
    if (action_index == -1){
        ROS_WARN_STREAM("Robot " << robot_id_ << ": Invalid input/output to neural net (NaN). Performing 0 action.");
        action << 0,0,0;
    } else if (action_index > -1) {
      // convert NN output to feasible action on the map
      action(0) = state(action_index, 0); // direction to go to
      action(1) = std::max(state(action_index, 2) - clearance_,
                           0.0);          // distance to travel into direction (frontier-clearance_ or zero)
      action(2) = state(action_index, 0); // new heading of the robot
    }
    // ROS_INFO_STREAM("Robot " << robot_id_ << ": Action (Eigen::Vector3d):\n" << action);

    int robot_move_towards_each_other = 0;
    for (int r = 0; r < state.rows(); ++r) {
      if (state(r, 3) != 0) {
        robot_move_towards_each_other = r;
      }
    }

    if (robot_move_towards_each_other == action_index) {
      ROS_INFO("\n\n=========== robot_move_towards_each_other: %d =====================\n\n",
               robot_move_towards_each_other);
    }
    std::ofstream action_log_file;
    action_log_file.open(log_dir + "/" + action_log_file_name, std::ios_base::app);
    action_log_file << ros::Time::now();
    action_log_file << "," + std::to_string(robot_move_towards_each_other == action_index);
    action_log_file << std::endl;

    goal.target_pose.pose = polarToPose(action, current_odom); // TODO: dereferencing occurring in correct order?
    return goal;
}

void ActionNode::getAction(const Eigen::MatrixXd state, int &action_index) {
  // This method outputs an action belonging to a state according to the loaded policy
  // ROS_INFO("Starting getAction()\n");
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> M(state.block(0, 1, n_th_, 1 + n_robots_));
  Eigen::Map<VectorXd> nn_input(M.data(), M.size());
  // ROS_INFO("M.size: (%ld, %ld) nn_input.size: (%ld, %ld)\n", M.rows(), M.cols(), nn_input.rows(), nn_input.cols());
  if (nn_input.maxCoeff() != 0) {
    nn_input = nn_input / nn_input.maxCoeff();                        // scale input
    Eigen::VectorXd nn_output = policy_->EvaluateNNSoftmax(nn_input); // Evaluate the network with the state as input
    // ROS_INFO_STREAM("Robot " << robot_id_ << ": NN output " << nn_output);
    // sample from NN output
    if (std::isnan(nn_output.norm()) || std::isinf(nn_output.norm())) {
      action_index = -1;
    } else {
      std::vector<double> weights;
      for (size_t i = 0; i < nn_output.size(); i++) {
        weights.push_back(nn_output(i));
      }
      std::discrete_distribution<int> distr(weights.begin(), weights.end());
      action_index = distr(generator_);
      ROS_INFO_STREAM("Robot " << robot_id_ << ": Selecting bearing " << action_index << " with probability "
                               << nn_output(action_index) << " as action");
    }
  } else {
    action_index = -1;
  }
}

void ActionNode::actionThread(){
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true) ;
    actionlib::SimpleClientGoalState *goal_state = nullptr; 
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_WARN_STREAM("Robot " << robot_id_ << ": Waiting for the move_base action server to come up");
    }
    Eigen::MatrixXd last_state(current_state_.rows(),current_state_.cols());
    int last_action_index = -1;
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
                        //ROS_INFO_STREAM("Robot " << robot_id_ << ": Ready. Getting next way point");
                        if (mode_==LEARNING){
                            savePolicy();
                            updatePolicy(last_state, last_action_index);
                        }
                        *goal_state = ac.sendGoalAndWait(getGoal(last_state, last_action_index));
                        if(*goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
                            ROS_INFO_STREAM("Robot " << robot_id_ << ": Waypoint reached.");
                        } else {
                            ROS_WARN_STREAM("Robot " << robot_id_ << ": The base failed to reach the waypoint.") ;
                        }
                    } else {
                        ROS_INFO_STREAM("Robot " << robot_id_ << ": goal_state is not done. Waiting...");
                    }
                } else {
                    ROS_INFO_STREAM("Robot " << robot_id_ << ": No action status available. Starting exploration");
                    if (mode_==LEARNING){
                        savePolicy();
                        updatePolicy(last_state, last_action_index);
                    }
                    goal_state = new actionlib::SimpleClientGoalState(ac.sendGoalAndWait(getGoal(last_state, last_action_index)));
                }
            } else {
                ROS_WARN_STREAM("Robot " << robot_id_ << ": No map/status/state available. Waiting...");
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