#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "geometry_msgs/PointStamped.h"
#include <ros/console.h>

typedef unsigned int UINT ;

class robot_costmap
{
  public:
    robot_costmap(ros::NodeHandle, costmap_2d::Costmap2DROS *) ;
    ~robot_costmap() {}
    
  private:
    ros::Subscriber subWaypoint ;
    ros::Subscriber subCostmapUpdate ;
    
    costmap_2d::Costmap2DROS * costmap_ros_ ;
    costmap_2d::Costmap2D *costmap_ ;
    geometry_msgs::PointStamped currentWaypoint ;
    bool fMap ;
    
    void waypointCallback(const geometry_msgs::PointStamped&) ;
    void mapUpdateCallback(const map_msgs::OccupancyGridUpdate&) ;
} ;

robot_costmap::robot_costmap(ros::NodeHandle nh, costmap_2d::Costmap2DROS *cm_ros_): costmap_ros_(cm_ros_), fMap(false){
  subWaypoint = nh.subscribe("/clicked_point", 10, &robot_costmap::waypointCallback, this) ;
  subCostmapUpdate = nh.subscribe("robot_map/robot_map/costmap_updates", 10, &robot_costmap::mapUpdateCallback, this) ;
}

void robot_costmap::waypointCallback(const geometry_msgs::PointStamped& msg){
  currentWaypoint = msg ;
  if (fMap){
    double wx = currentWaypoint.point.x ;
    double wy = currentWaypoint.point.y ;
    UINT mx ;
    UINT my ;
    bool convSuccess = costmap_->worldToMap(wx,wy,mx,my) ;
    if (!convSuccess)
      ROS_INFO_STREAM("Waypoint out of costmap bounds!") ;
    else {
      unsigned char wpCellCost = costmap_->getCost(mx,my) ;
      ROS_INFO_STREAM("Waypoint cell cost: " << (int)wpCellCost) ;
    }
  }
}

void robot_costmap::mapUpdateCallback(const map_msgs::OccupancyGridUpdate&){
  costmap_ = costmap_ros_->getCostmap() ;
  fMap = true ;
}
