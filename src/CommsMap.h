#include <vector>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

using std::vector ;
using std::string ;

class CommsMap{
  public:
    CommsMap(ros::NodeHandle) ;
    ~CommsMap(){}
  private:
    vector<ros::Subscriber> sub_local_maps_ ;
    vector<ros::Subscriber> sub_true_poses_ ;
    vector<ros::Publisher> pub_shared_maps_ ;
    
    void mapCallback(const nav_msgs::OccupancyGrid&) ;
    void poseCallback(const nav_msgs::Odometry&) ;
    double euclideanDistance(vector<double>, vector<double>) ;
    
    size_t r_idx ;                              // this robot index
    size_t n_robots ;                           // number of robots
    double comms_range ;                        // communication range
    vector< vector<double> > r_poses ;          // latest true pose
    vector<bool> f_pose ;                       // true if first true pose has been received
    vector<bool> f_map ;                        // true if latest maps need to be published
    vector<bool> f_map0 ;                       // true if first map need to be published
    vector<nav_msgs::OccupancyGrid> shared_map ;// latest maps
} ;

CommsMap::CommsMap(ros::NodeHandle nh){
  double tmp ;
  ros::param::get("comms_map/robot_index", tmp) ;
  r_idx = (size_t)tmp ;
  
  ros::param::get("comms_map/num_robots", tmp) ;
  n_robots = (size_t)tmp ;
  
  ros::param::get("comms_map/comms_range", comms_range) ;
  
  ROS_INFO_STREAM("Creating map republishers for " << n_robots << " robots") ;
  ROS_INFO_STREAM("Communication range set to " << comms_range) ;
  
  for (size_t i = 0; i < n_robots; i++){
    char buffer[50] ;
    sprintf(buffer,"/robot_%lu/map",i) ;
    sub_local_maps_.push_back(nh.subscribe(buffer, 10, &CommsMap::mapCallback, this)) ;
    
    sprintf(buffer, "/robot_%lu/base_pose_ground_truth",i) ;
    sub_true_poses_.push_back(nh.subscribe(buffer, 10, &CommsMap::poseCallback, this)) ;
    
    sprintf(buffer,"comms_map/robot_%lu/map",i) ;
    pub_shared_maps_.push_back(nh.advertise<nav_msgs::OccupancyGrid>(buffer, 10, true)) ;
    
    f_pose.push_back(false) ;
    f_map.push_back(false) ;
    f_map0.push_back(true) ;
    
    r_poses.push_back(vector<double>(2,0.0)) ;
    nav_msgs::OccupancyGrid tmp ;
    shared_map.push_back(tmp) ;
  }
}

void CommsMap::mapCallback(const nav_msgs::OccupancyGrid& msg){
  string frame_id = msg.header.frame_id ;
  string delim("/") ;
  
  size_t j ;
  if (frame_id.compare(0,1,delim) == 0){ // frame_id starts with "/robot_"
    j = 7 ;
  }
  else{ // frame_id starts with "robot_"
    j = 6 ;
  }
  
  size_t n_len = 0 ;
  for (size_t i = j; i < frame_id.length(); i++){
    if (frame_id.compare(i,1,delim) == 0){
      break ;
    }
    n_len++ ;
  }
  
  for (size_t i = 0; i < n_robots; i++){
    char key[50] ;
    sprintf(key, "%lu", i) ;
    if (frame_id.compare(j,n_len,key) == 0){
      shared_map[i] = msg ;
      f_map[i] = true ; // new map recorded and needs to be published when within range
      if (f_map0[i]){ // always share the first map
        ROS_INFO_STREAM("robot_" << r_idx << ": Sharing first map from robot_" << i ) ;
        pub_shared_maps_[i].publish(shared_map[i]) ;
        f_map0[i] = false ;
        f_map[i] = false ;
        break ;
      }
    }
  }
}

void CommsMap::poseCallback(const nav_msgs::Odometry& msg){
  string frame_id = msg.header.frame_id ;
  string delim("/") ;
  size_t j ;
  if (frame_id.compare(0,1,delim) == 0){ // frame_id starts with "/"
    j = 7 ;
  }
  else{
    j = 6 ;
  }
  
  size_t n_len = 0 ;
  for (size_t i = j; i < frame_id.length(); i++){
    if (frame_id.compare(i,1,delim) == 0){
      break ;
    }
    n_len++ ;
  }
  
  for (size_t i = 0; i < n_robots; i++){
    char key[50] ;
    sprintf(key, "%lu", i) ;
    if (frame_id.compare(j,n_len,key) == 0){
      r_poses[i][0] = msg.pose.pose.position.x ;
      r_poses[i][1] = msg.pose.pose.position.y ;
      
      if (!f_pose[i]){
        f_pose[i] = true ;
      }
      
      if (f_pose[i] && f_pose[r_idx]){ // if ground truth poses available for both robots
        double d = euclideanDistance(r_poses[i], r_poses[r_idx]) ;
        
        if (d <= comms_range && f_map[i]){ // if within comms range and need to share map
          if (i != r_idx){
            ROS_INFO_STREAM("robot_" << r_idx << ": Receiving map from robot_" << i ) ;
          }
          pub_shared_maps_[i].publish(shared_map[i]) ;
          f_map[i] = false ; // latest map has been published
        }
      }
      break ;
    }
  }
  
  
}

double CommsMap::euclideanDistance(vector<double> a, vector<double> b){
  double dx = pow(a[0] - b[0], 2) ;
  double dy = pow(a[1] - b[1],2) ;
  double d = sqrt(dx + dy) ;
  return d ;
}
