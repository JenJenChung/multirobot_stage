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

geometry_msgs::Pose2D odomToPose2D(nav_msgs::Odometry odom){
    geometry_msgs::Pose2D pose;
    pose.x = odom.pose.pose.position.x;
    pose.y = odom.pose.pose.position.y;
    Eigen::Quaterniond quat;
    tf::quaternionMsgToEigen(odom.pose.pose.orientation, quat);
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2,1,0);
    pose.theta = euler[0];
    return pose;
}

Eigen::MatrixXd mapToPolar(const nav_msgs::OccupancyGrid &map, nav_msgs::Odometry *odoms, int robot_id, int n_robots, int n_th=4, double thresh=50){
    Eigen::MatrixXd polar = Eigen::MatrixXd::Zero(n_th,2+n_robots);
    double res = map.info.resolution;
    geometry_msgs::Point origin = map.info.origin.position;
    geometry_msgs::Pose2D robot = odomToPose2D(odoms[robot_id]);
    double theta;
    int l = 0;
    int b = 0;
    bool looking = true;
    for (int i = 0; i<n_th; i++){
        theta = theta + 2*M_PI/n_th;
        l = 0;
        b = 0;
        looking = true;
        polar(i,0) = theta;
        while (looking){
            int x = round(cos(robot.theta + theta)*l + (robot.x-origin.x)/res);
            int y = round(sin(robot.theta + theta)*l + (robot.y-origin.y)/res);
            if (x > map.info.width || y > map.info.height || x < 0 || y < 0){
                looking = false;
            } else {
                int a = y*map.info.width + x;
                if (map.data[a]>thresh && polar(i,1)==0){ // distance to closest obstacle
                    polar(i,1) = l*res;
                }
                if (map.data[a]==-1 && map.data[b]!=-1){ // distance to frontier
                    polar(i,2) = l*res;
                }
                b = a;
                l++;   
            }
        }
    }
    // add the other robots, one column per robot.
    int r_idx = 3;
    for (int r = 0; r<n_robots; r++){
        if (r!=robot_id){
            geometry_msgs::Pose2D other_robot = odomToPose2D(odoms[r]);
            double dx = other_robot.x-robot.x;
            double dy = other_robot.y-robot.y;
            double l_r = sqrt(pow(dx,2)+pow(dy,2)); // distance to current robot
            double t_r = atan2(dy,dx)-robot.theta; // bearing to 
            int t_idx = round(t_r*n_th/(2*M_PI));
            polar(t_idx,r_idx) = l_r;
            r_idx++;
        }
    }
    return polar;
}

geometry_msgs::Twist polarToTwist(const Eigen::Vector3d &polar, nav_msgs::Odometry &odom){
    geometry_msgs::Pose2D robot = odomToPose2D(odom);
    geometry_msgs::Twist out;
    out.linear.x = cos(polar(0) + robot.theta) * polar(1) + robot.x;
    out.linear.y = sin(polar(0) + robot.theta) * polar(1) + robot.y;
    out.linear.z = 0;
    out.angular.x = 0;
    out.angular.y = 0;
    out.angular.z = std::fmod(180/M_PI*(polar(2) + robot.theta),360);
    return out;
}

geometry_msgs::Pose polarToPose(const Eigen::Vector3d &polar, nav_msgs::Odometry &odom){
    geometry_msgs::Pose2D robot = odomToPose2D(odom);
    geometry_msgs::Pose pose;
    pose.position.x = cos(polar(0) + robot.theta) * polar(1) + robot.x;
    pose.position.y = sin(polar(0) + robot.theta) * polar(1) + robot.y;
    pose.position.z = 0;
    tf::Quaternion quat;  
    quat.setEuler(0, 0, std::fmod(180/M_PI*(polar(2) + robot.theta),360));
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
}

visualization_msgs::MarkerArray polarToMarkerArray(const Eigen::MatrixXd &polar, nav_msgs::Odometry &odom, double res=0.005){
    geometry_msgs::Pose2D robot = odomToPose2D(odom);
    visualization_msgs::MarkerArray markers;
    double obstx, obsty, frontx, fronty;
    visualization_msgs::Marker obst, front;
    for (int i=0; i<polar.rows(); i++){
        obst.ns = "obstacle_markers";
        obst.id = i;
        obst.header.frame_id = odom.header.frame_id;
        obst.header.stamp = ros::Time();
        obst.pose.position.x = cos(robot.theta + polar(i,0))*polar(i,1) + robot.x;
        obst.pose.position.y = sin(robot.theta + polar(i,0))*polar(i,1) + robot.y;
        obst.pose.orientation.w = 1;
        obst.action = visualization_msgs::Marker::ADD;
        obst.type = visualization_msgs::Marker::SPHERE;
        obst.scale.x = 20*res;
        obst.scale.y = 20*res;
        obst.scale.z = 20*res; 
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
        front.scale.x = 20*res;
        front.scale.y = 20*res;
        front.scale.z = 20*res;
        front.color.r = 0;
        front.color.g = 0;
        front.color.b = 1;
        front.color.a = 1;
        front.frame_locked = true;

        markers.markers.push_back(obst);
        markers.markers.push_back(front);
    }
    return markers;
}

#endif // CONVERSIONS_H_