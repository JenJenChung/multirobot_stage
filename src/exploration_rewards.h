#include <ros/ros.h>
#include <ros/console.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32.h"

// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <tf/transform_datatypes.h>

class ExplorationRewards {

    public:
      ExplorationRewards() {
        _sub = _nHandle.subscribe("/map_merge/map", 10, &ExplorationRewards::mapCallback, this);
        _pub = _nHandle.advertise<std_msgs::Float32>("explored_area", 10);
      }
      ~ExplorationRewards()
      {
          
      }

      void mapCallback(const nav_msgs::OccupancyGrid &msg);

    private:
      ros::NodeHandle _nHandle;
      ros::Subscriber _sub;
      ros::Publisher _pub;
};
