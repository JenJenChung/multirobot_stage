#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher pub ;

void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  nav_msgs::OccupancyGrid map = msg ;
  
  double xmin, ymin ;
  ros::param::get("gmapping/xmin", xmin) ;
  ros::param::get("gmapping/ymin", ymin) ;
  
  map.info.origin.position.x = xmin ;
  map.info.origin.position.y = ymin ;
  map.info.origin.orientation.w = 1.0 ;
  
  pub.publish(map) ;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "merged_map_republisher") ;
  ros::NodeHandle nh ;
  
  ros::Subscriber sub = nh.subscribe("raw_merged_map", 10, &mapCallback) ;
  pub = nh.advertise<nav_msgs::OccupancyGrid>("merged_map", 10) ;
  
  ros::spin() ;

  return 0 ;
}
