# ifndef CONVERSIONS_H_
# define CONVERSIONS_H_


# include <ros/ros.h>
# include <math.h>
# include <algorithm> 
# include <nav_msgs/OccupancyGrid.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/Pose2D.h>
# include <geometry_msgs/Twist.h>
# include <Eigen/Dense>
# include <eigen_conversions/eigen_msg.h>
# include <tf/transform_datatypes.h>
# include <visualization_msgs/MarkerArray.h>

geometry_msgs::Pose2D poseToPose2D(geometry_msgs::Pose pose){
    geometry_msgs::Pose2D pose2D;
    pose2D.x = pose.position.x;
    pose2D.y = pose.position.y;
    Eigen::Quaterniond quat;
    tf::quaternionMsgToEigen(pose.orientation, quat);
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2,1,0);
    pose2D.theta = euler(0);
    return pose2D;
}

geometry_msgs::Pose2D odomToPose2D(nav_msgs::Odometry odom){
    return poseToPose2D(odom.pose.pose); 
}

Eigen::MatrixXd mapToPolar(const nav_msgs::OccupancyGrid &map, std::vector<std::shared_ptr<nav_msgs::Odometry>> odomptrs, int robot_id, int n_th, double thresh=50){
    int n_robots = odomptrs.size();
    std::vector<nav_msgs::Odometry> odoms(n_robots);
    for (std::size_t i = 0; i<odoms.size(); i++){
        odoms[i] = *(odomptrs[i]);
    } 
    
    Eigen::MatrixXd polar = Eigen::MatrixXd::Zero(n_th,2+n_robots);
    double res = map.info.resolution;
    geometry_msgs::Point origin = map.info.origin.position;
    geometry_msgs::Pose2D robot = odomToPose2D(odoms[robot_id]);
    double theta = 0;
    std::size_t l = 0;
    std::size_t b = 0;
    bool looking = true;
    for (std::size_t i = 0; i<n_th; i++){
        l = 0;
        b = 0;
        looking = true;
        polar(i,0) = theta;
        while (looking){
            std::size_t x = round(cos(robot.theta + theta)*l + (robot.x-origin.x)/res);
            std::size_t y = round(sin(robot.theta + theta)*l + (robot.y-origin.y)/res);
            if (x > map.info.width || y > map.info.height || x < 0 || y < 0){ // reached end of map
                looking = false;
            } else {
                std::size_t a = y*map.info.width + x;
                if (map.data[a]>thresh && polar(i,1)==0){ // distance to closest obstacle
                    polar(i,1) = l*res;
                }
                if (map.data[a]==-1 && map.data[b]==0){ // distance to frontier
                    polar(i,2) = l*res;
                }
                b = a;
                l++;   
            }
        }
        theta = theta + 2*M_PI/n_th;
    }
    // add the other robots, one column per robot.
    std::size_t r_idx = 3;
    for (std::size_t r = 0; r<n_robots; r++){
        if (r!=robot_id){
            geometry_msgs::Pose2D other_robot = odomToPose2D(odoms[r]);
            double dx = other_robot.x-robot.x;
            double dy = other_robot.y-robot.y;
            double l_r = sqrt(pow(dx,2)+pow(dy,2)); // distance to current robot
            double t_r = atan2(dy,dx)-robot.theta; // bearing to current robot
            std::size_t t_idx = std::fmod(t_r*n_th/(2*M_PI)+n_th,n_th); // make sure index is between 0 and n_th-1
            // ROS_INFO("[Robot-%i-mapToPolar] Index of robot %lu in the table: %lu,%lu", robot_id, r, t_idx, r_idx);
            polar(t_idx,r_idx) = l_r;
            r_idx++;
        }
    }
    return polar;
}

// geometry_msgs::Twist polarToTwist(const Eigen::Vector3d polar, nav_msgs::Odometry odom){
//     geometry_msgs::Pose2D robot = odomToPose2D(odom);
//     geometry_msgs::Twist out;
//     out.linear.x = cos(polar(0) + robot.theta) * polar(1) + robot.x;
//     out.linear.y = sin(polar(0) + robot.theta) * polar(1) + robot.y;
//     out.linear.z = 0;
//     out.angular.x = 0;
//     out.angular.y = 0;
//     out.angular.z = std::fmod(180/M_PI*(polar(2) + robot.theta),360);
//     return out;
// }

geometry_msgs::Pose polarToPose(const Eigen::Vector3d polar, nav_msgs::Odometry odom){
    geometry_msgs::Pose2D robot = odomToPose2D(odom);
    //std::cout << "[polarToPose] odom:\n" << odom.pose.pose << std::endl << "robot:\n" << robot << std::endl;
    geometry_msgs::Pose pose;
    pose.position.x = cos(polar(0) + robot.theta) * polar(1) + robot.x;
    pose.position.y = sin(polar(0) + robot.theta) * polar(1) + robot.y;
    pose.position.z = 0;
    tf::Quaternion quat;
    quat.setEuler(0,0,std::fmod(polar(2) + robot.theta,2*M_PI));
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    //std::cout << "[polarToPose] pose:\n" << pose << std::endl << "robot:\n" << poseToPose2D(pose) << std::endl;
    return pose;
}

visualization_msgs::MarkerArray polarToMarkerArray(const Eigen::MatrixXd polar, nav_msgs::Odometry odom, double res=0.005){
    geometry_msgs::Pose2D robot = odomToPose2D(odom);
    visualization_msgs::MarkerArray markers;
    double obstx, obsty, frontx, fronty;
    visualization_msgs::Marker obst, front, rob;
    for (std::size_t i=0; i<polar.rows(); i++){
        obst.ns = "obstacle_markers";
        obst.id = i;
        obst.header.frame_id = odom.header.frame_id;
        obst.header.stamp = ros::Time();
        obst.pose.position.x = cos(robot.theta + polar(i,0))*polar(i,1) + robot.x;
        obst.pose.position.y = sin(robot.theta + polar(i,0))*polar(i,1) + robot.y;
        obst.pose.orientation.w = 1;
        obst.action = visualization_msgs::Marker::ADD;
        obst.type = visualization_msgs::Marker::SPHERE;
        obst.scale.x = 25*res;
        obst.scale.y = 25*res;
        obst.scale.z = 25*res; 
        obst.color.r = 1;
        obst.color.g = 0;
        obst.color.b = 0;
        obst.color.a = 1;
        obst.frame_locked = true;

        front.ns = "frontier_markers";
        front.id = i;
        front.header.frame_id = odom.header.frame_id;
        front.header.stamp = ros::Time();
        front.pose.position.x = cos(robot.theta + polar(i,0))*polar(i,2) + robot.x;
        front.pose.position.y = sin(robot.theta + polar(i,0))*polar(i,2) + robot.y;
        front.pose.orientation.w = 1;
        front.action = visualization_msgs::Marker::ADD;
        front.type = visualization_msgs::Marker::SPHERE;
        front.scale.x = 25*res;
        front.scale.y = 25*res;
        front.scale.z = 25*res;
        front.color.r = 0;
        front.color.g = 0;
        front.color.b = 1;
        front.color.a = 1;
        front.frame_locked = true;

        markers.markers.push_back(obst);
        markers.markers.push_back(front);

        if (polar.cols()==4 && polar(i,3)!=0){
            rob.ns = "robot_markers";
            rob.id = 0;
            rob.header.frame_id = odom.header.frame_id;
            rob.header.stamp = ros::Time();
            rob.pose.position.x = cos(robot.theta + polar(i,0))*polar(i,3) + robot.x;
            rob.pose.position.y = sin(robot.theta + polar(i,0))*polar(i,3) + robot.y;
            rob.pose.orientation.w = 1;
            rob.action = visualization_msgs::Marker::ADD;
            rob.type = visualization_msgs::Marker::SPHERE;
            rob.scale.x = 50*res;
            rob.scale.y = 50*res;
            rob.scale.z = 50*res;
            rob.color.r = 0;
            rob.color.g = 1;
            rob.color.b = 0;
            rob.color.a = 1;
            rob.frame_locked = true;
            markers.markers.push_back(rob);
        }
    }
    return markers;
}

#endif // CONVERSIONS_H_