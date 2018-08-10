#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/console.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "multirobot_stage/NeuralNet.h"

#include "multirobot_stage/RobotPosition.h"
#include "map_preprocessor/MapEncoding.h"

using namespace Eigen ;
using std::vector ;
using std::string ;

class AgentPolicy{
  public:
    AgentPolicy(ros::NodeHandle nh) ;
    ~AgentPolicy() ;
  private:
    ros::Subscriber sub_global_odom_ ;      // robot odometry in global frame
    ros::Subscriber sub_map_code_ ;         // local map encoding
    vector<ros::Subscriber> sub_rob_pose_ ; // poses of other robots
    ros::Subscriber sub_result_ ;           // move_base action result
    
    ros::Publisher pub_map_goal_ ;          // publish a goal position in the map frame
    
    void globalOdomCallback(const nav_msgs::Odometry&) ;
    void mapCodeCallback(const map_preprocessor::MapEncoding&) ;
    void robPoseCallback(const multirobot_stage::RobotPosition&) ;
    void waypointCallback(const move_base_msgs::MoveBaseActionResult&) ;
    
    vector<double> polarToPoint(Vector2d) ; // convert from polar coordinates to (x,y)
    
    int code_size ;       // map encoding size
    int num_agents ;      // total number of agents
    int r_idx ;           // this agent's index
    string root_ns ;      // root namespace, default "robot"
    
    nav_msgs::Odometry global_odom ;  // latest odometry in global frame
    vector<double> agent_state ;      // [map encoding, relative poses]
    NeuralNet * NNPolicy ;            // agent's neural network policy
    
    bool f_slam ;           // first global position has been received
    bool f_map ;            // first map encoding has been received
    vector<bool> f_poses ;  // first set of relative poses has been received
} ;

AgentPolicy::AgentPolicy(ros::NodeHandle nh){
  ros::param::get("encoding_size", code_size) ;
  ros::param::get("num_agents", num_agents) ;
  ros::param::get("root_namespace", root_ns) ;
  ros::param::get("robot_index", r_idx) ;
  
  ROS_INFO_STREAM("Initialising policy for " << root_ns << "_" << r_idx << "...") ;
  
  sub_global_odom_ = nh.subscribe("global_odom", 10, &AgentPolicy::globalOdomCallback, this) ;
  sub_map_code_ = nh.subscribe("map_encoding", 10, &AgentPolicy::mapCodeCallback, this) ;
  sub_result_ = nh.subscribe("move_base/result", 10, &AgentPolicy::waypointCallback, this) ;
  pub_map_goal_ = nh.advertise<geometry_msgs::Twist>("map_goal", 10, true) ;
  
  ROS_INFO_STREAM("  " << root_ns << "_" << r_idx << ": subscribing to " << num_agents-1 << " robot poses...") ;
  for (int i = 0; i < num_agents; i++){
    if (i != r_idx){
      char buffer[50] ;
      sprintf(buffer, "/%s_%i/position", root_ns.c_str(), i) ;
      sub_rob_pose_.push_back(nh.subscribe(buffer, 10, &AgentPolicy::robPoseCallback, this)) ;
    }
  }
  
  // Initialise agent state
  int agent_state_size = code_size + 2*(num_agents-1) ;
  vector<double> tmp(agent_state_size, 0.0) ;
  agent_state = tmp ;
  
  // Initialise flags
  f_slam = false ;
  f_map = false ;
  vector<bool> f_tmp(num_agents-1,false) ;
  f_poses = f_tmp ;
  
  // Initialise agent's neural network policy
  ROS_INFO_STREAM("  " << root_ns << "_" << r_idx << ": Initialising NN policy...") ;
  size_t num_out = 2 ; // [true bearing, distance]
  size_t num_hidden = agent_state_size * 2 ;
  
  NNPolicy = new NeuralNet(agent_state_size, num_out, num_hidden, LOGISTIC) ;
  
  // NN weights
  vector<double> AA, BB ;
  char buffer[50] ;
  sprintf(buffer, "/%s_%i/A", root_ns.c_str(), r_idx) ;
  if (!ros::param::has(buffer)){
    ROS_INFO_STREAM("  " << root_ns << "_" << r_idx << ": No NN policy weights found, initialising with random values.") ;
  }
  else{
    ros::param::get(buffer, AA) ;
    sprintf(buffer, "/%s_%i/B", root_ns.c_str(), r_idx) ;
    if (!ros::param::has(buffer)){
      ROS_INFO_STREAM("  " << root_ns << "_" << r_idx << ": No NN policy weights found, initialising with random values.") ;
    }
    else{ // Both A and B matrix weights found for initialisation
      ros::param::get(buffer, BB) ;
  
      MatrixXd A(agent_state_size, num_hidden) ;
      MatrixXd B(num_hidden+1, num_out) ;
      
      size_t ii = 0 ;
      for (size_t i = 0; i < A.rows(); i++){
        for (size_t j = 0; j < A.cols(); j++){
          A(i,j) = AA[ii++] ;
        }
      }
      
      ii = 0 ;
      for (size_t i = 0; i < B.rows(); i++){
        for (size_t j = 0; j < B.cols(); j++){
          B(i,j) = BB[ii++] ;
        }
      }
      NNPolicy->SetWeights(A, B) ;
    }
  }
  ROS_INFO_STREAM("  " << root_ns << "_" << r_idx << ": NN policy initialised!") ;
}

AgentPolicy::~AgentPolicy(){
  delete NNPolicy ;
  NNPolicy = 0 ;
}

void AgentPolicy::globalOdomCallback(const nav_msgs::Odometry& msg){
  global_odom = msg ;
  if (!f_slam){
    f_slam = true ;
  }
}

void AgentPolicy::mapCodeCallback(const map_preprocessor::MapEncoding& msg){
  for (size_t i = 0; i < msg.data.size(); i++){
    agent_state[i] = msg.data[i] ;
  }
  if (!f_map){
    f_map = true ;
  }
}

void AgentPolicy::robPoseCallback(const multirobot_stage::RobotPosition& msg){
  int idx = msg.idx ;
  
  // Compute index in state vector
  int i ;
  if (idx < r_idx){
    i = idx ;
  }
  else{
    i = idx - 1 ;
  }
  int j = code_size + i*2 ;
  
 // Store as absolute positions (conversion only in waypointCallback)
  agent_state[j] = msg.position.x ;
  agent_state[j+1] = msg.position.y ;

  if (!f_poses[i]){
    f_poses[i] = true ;
  }
}

void AgentPolicy::waypointCallback(const move_base_msgs::MoveBaseActionResult& msg){
  if (msg.status.status == 2 || msg.status.status == 4 || msg.status.status == 5 || msg.status.status == 6){ // waypoint was preempted or aborted
    ROS_INFO_STREAM(root_ns << "_" << r_idx << "Unable to reach waypoint! Selecting new waypoint...") ;
  }
  
  // Check all necessary data is available
  if (!f_slam){
    ROS_INFO_STREAM(root_ns << "_" << r_idx << "No odometry received yet! Waiting for first global_odom message...") ;
    return ;
  }
  
  if (!f_map){
    ROS_INFO_STREAM(root_ns << "_" << r_idx << "No map received yet! Waiting for first map message...") ;
    return ;
  }
  
  bool f_pose_all = true ;
  for (size_t i = 0; i < f_poses.size(); i++){
    if (!f_poses[i]){
      size_t j ;
      if (i < r_idx){
        j = i ;
      }
      else{
        j = i+1 ;
      }
      ROS_INFO_STREAM(root_ns << "_" << r_idx << ": " << root_ns << "_" << j << " position unknown! Waiting for first position message...") ;
      f_pose_all = false ;
    }
  }
  
  if (!f_pose_all){
    return ;
  }
  
  // Compute policy input state
  VectorXd input_state(agent_state.size()) ;
  for (size_t i = 0; i < agent_state.size(); i++){
    input_state(i) = agent_state[i] ;
  }
  
  // Compute relative polar coordinates in agent state
  for (size_t i = 0; i < f_poses.size(); i++){
    int j  = code_size + i*2 ;
    double dx = agent_state[j] - global_odom.pose.pose.position.x ;
    double dy = agent_state[j+1] - global_odom.pose.pose.position.y ;
    input_state(j) = sqrt(pow(dx,2) + pow(dy,2)) ;
    input_state(j+1) = atan2(dy, dx) ;
  }
  
  // Compute agent policy output
  Vector2d action = NNPolicy->EvaluateNN(input_state) ;
  
  // Convert to global coordinate frame
  vector<double> p = polarToPoint(action) ;
  
  geometry_msgs::Twist waypoint ;
  
  waypoint.linear.x = p[0] + global_odom.pose.pose.position.x ;
  waypoint.linear.y = p[1] + global_odom.pose.pose.position.y ;
  waypoint.linear.z = 0 ;
  waypoint.angular.x = 0 ;
  waypoint.angular.y = 0 ;
  waypoint.angular.z = 0 ;
  
  pub_map_goal_.publish(waypoint) ;
}

vector<double> AgentPolicy::polarToPoint(Vector2d a){
  vector<double> p ;
  p.push_back(a[0] * cos(a[1])) ;
  p.push_back(a[0] * sin(a[1])) ;
  return p ;
}
