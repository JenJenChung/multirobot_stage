#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <math.h> // M_PI
#include <algorithm>

using std::min ;

class bouncer{
  public:
    bouncer(ros::NodeHandle) ;
    ~bouncer() {}
  private:
    ros::Subscriber subScan ;
    ros::Publisher pubCmdvel ;

    double distanceThreshold ;
    double forwardVel ;
    double headingVel ;

    void scanCallback(const sensor_msgs::LaserScan&) ;
};

bouncer::bouncer(ros::NodeHandle nh){
  subScan = nh.subscribe("base_scan", 10, &bouncer::scanCallback, this) ;
  pubCmdvel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50, true) ;
  
  double temp = 0.0 ;
  ros::param::get("bouncer/distance_threshold", temp) ;
  distanceThreshold = temp ;
  ros::param::get("bouncer/forward_velocity", temp) ;
  forwardVel = temp ;
}

void bouncer::scanCallback(const sensor_msgs::LaserScan& msg){
  double currentAngle = msg.angle_min ;
  double angleIncrement = msg.angle_increment ;
  
  headingVel = 0.0 ;
  double minScan = msg.ranges[0] ;
  for (size_t i = 0; i < msg.ranges.size(); i++){
    if (currentAngle > -M_PI/2.0 && currentAngle < 0.0 && msg.ranges[i] < distanceThreshold)
      headingVel = 1.0 ; // Turn left
    else if (currentAngle >= 0.0 && currentAngle < M_PI/2.0 && msg.ranges[i] < distanceThreshold)
      headingVel = -1.0 ; // Turn right
    
    if (msg.ranges[i] < minScan)
      minScan = msg.ranges[i] ;
    
    currentAngle += angleIncrement ;
  }
  
  // Set the robot commanded velocities
  geometry_msgs::Twist cmd ;
  
  // Slow down if within threshold distance
  cmd.linear.x = forwardVel*min(1.0,minScan/distanceThreshold) ;
  cmd.linear.y = 0.0 ;
  cmd.linear.z = 0.0 ;
  cmd.angular.x = 0.0 ;
  cmd.angular.y = 0.0 ;
  cmd.angular.z = headingVel ;
  
  pubCmdvel.publish(cmd) ;
}
