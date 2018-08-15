#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class GlobalOdom{
  public:
    GlobalOdom(ros::NodeHandle) ;
    ~GlobalOdom(){}
  private:
    ros::Subscriber sub_odom_ ;
    ros::Publisher pub_global_odom_ ; 

    tf2_ros::Buffer tf_buffer_ ;
    tf2_ros::TransformListener tf_listener_ ;
    
    std::string map_frame ;
    
    void odomCallback(const nav_msgs::Odometry&) ;
} ;

GlobalOdom::GlobalOdom(ros::NodeHandle nh): tf_listener_(tf_buffer_){
  if (ros::param::has("global_frame")){
    ros::param::get("global_frame", map_frame) ;
  }
  else{
    map_frame = "map" ;
  }
  
  sub_odom_ = nh.subscribe("odom", 10, &GlobalOdom::odomCallback, this) ;
  pub_global_odom_ = nh.advertise<nav_msgs::Odometry>("global_odom", 10) ;
}

void GlobalOdom::odomCallback(const nav_msgs::Odometry& msg){
  std::string odom_frame = msg.header.frame_id ;
  
  if (map_frame[0]=='/'){
    map_frame.erase(0,1);
  }
  if (odom_frame[0]=='/'){
    odom_frame.erase(0,1);
  }
  
  try{
    geometry_msgs::TransformStamped odom_trans = tf_buffer_.lookupTransform(map_frame, odom_frame, ros::Time(0)) ;
    geometry_msgs::PoseStamped pose_in, pose_out ;
    pose_in.header = msg.header ;
    pose_in.pose = msg.pose.pose ;
    tf2::doTransform(pose_in, pose_out, odom_trans) ;
    
    nav_msgs::Odometry msg_out ;
    msg_out.pose.pose = pose_out.pose ;
    msg_out.header = pose_out.header ;
    msg_out.child_frame_id = odom_frame ;
    pub_global_odom_.publish(msg_out) ;
  }
  catch (tf2::TransformException &ex){
    ROS_WARN("%s", ex.what()) ;
    ros::Duration(1.0).sleep() ;
  }
}
