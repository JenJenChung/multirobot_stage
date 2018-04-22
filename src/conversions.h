# include <ros/ros.h>
# include <math.h>
# include <algorithm> 
# include <nav_msgs/OccupancyGrid.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/Pose2D.h>
# include <geometry_msgs/Twist.h>
# include <Eigen/Dense>
# include <eigen_conversions/eigen_msg.h>

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


Eigen::MatrixXd mapToPolar(const nav_msgs::OccupancyGrid &map, nav_msgs::Odometry *odoms, int robot_id, int nRobots, int n_th=4, double thresh=50){
    Eigen::MatrixXd coff = Eigen::MatrixXd::Zero(n_th,2+nRobots);
    double res = map.info.resolution;
    geometry_msgs::Point origin = map.info.origin.position;
    geometry_msgs::Pose2D robot = odomToPose2D(odoms[robot_id]);
    int i = 0;
    for (double theta = 0; theta<2*M_PI; theta = theta + 2*M_PI/n_th ){
        int l = 0;
        int b = 0;
        bool looking = true;
        coff(i,0) = theta;
        while (looking){
            int x = round(cos(robot.theta + theta)*l + (robot.x-origin.x)/res);
            int y = round(sin(robot.theta + theta)*l + (robot.y-origin.y)/res);
            if (x > map.info.width || y > map.info.height || x < 0 || y < 0){
                looking = false;
            } else {
                int a = x*map.info.width + y;
                if (map.data[a]>thresh && coff(i,1)==0){
                    coff(i,1) = l*res;
                }
                if (map.data[a]==-1 && map.data[b]!=-1){
                    coff(i,2) = l*res;
                }
                b = a;
                l++;   
            }
        }
        i++;
    }
    // add the other robots, one column per robot.
    int r_idx = 0;
    for (int r = 0; r<nRobots; r++){
        if (r!=robot_id){
            geometry_msgs::Pose2D other_robot = odomToPose2D(odoms[r]);
            double dx = other_robot.x-robot.x;
            double dy = other_robot.y-robot.y;
            double l_r = sqrt(pow(dx,2)+pow(dy,2)); // distance to current robot
            double t_r = atan2(dy,dx)-robot.theta; // bearing to 
            int t_idx = round(t_r*n_th/(2*M_PI));
            coff(t_idx,r_idx) = l_r;
            r_idx++;
        }
    }

    return coff;
}

geometry_msgs::Twist polarToTwist(const Eigen::Vector2d &coff, nav_msgs::Odometry &odom){
    geometry_msgs::Pose2D robot = odomToPose2D(odom);
    geometry_msgs::Twist out;
    out.linear.x = cos(coff(0) + robot.theta) * coff(1);
    out.linear.y = sin(coff(0) + robot.theta) * coff(1);
    out.linear.z = 0;
    out.angular.x = 0;
    out.angular.y = 0;
    out.angular.z = 180/M_PI*(coff(2) + robot.theta);
    return out;
}


/*
THE FOLLWOING CODE IS NOT TESTED AND PROBABLY DOESNT WORK
nav_msgs::OccupancyGrid polar2map(const Eigen::MatrixXd &coff, geometry_msgs::Pose2D robot, double res=0.005){
    nav_msgs::OccupancyGrid map;
    map.info.resolution = res;
    
    std::vector<double> obstx, obsty, frontx, fronty;
    for (int i=0; i<coff.rows(); i++){
        obstx.push_back(cos(robot.theta + coff(i,0))*coff(i,1) + robot.x);
        obsty.push_back(sin(robot.theta + coff(i,0))*coff(i,1) + robot.y);
        frontx.push_back(cos(robot.theta + coff(i,0))*coff(i,2) + robot.x);
        fronty.push_back(sin(robot.theta + coff(i,0))*coff(i,2) + robot.y);
    }
    double maxx, minx, maxy, miny;
    maxy= std::max(*std::max_element(obsty.begin(),obsty.end()),   *std::max_element(fronty.begin(),fronty.end()));
    miny= std::min(*std::min_element(obsty.begin(),obsty.end()),   *std::min_element(fronty.begin(),fronty.end()));
    maxx= std::max(*std::max_element(obstx.begin(),obstx.end()),   *std::max_element(frontx.begin(),frontx.end()));
    minx= std::min(*std::min_element(obstx.begin(),obstx.end()),   *std::min_element(frontx.begin(),frontx.end()));
    int height = round((maxy-miny)/res);
    int width  = round((maxx-minx)/res);
    map.info.height = height;
    map.info.width  = width;

    std::vector<signed char> data(height*width); // = new std_msgs::Int8[height*width];
    for (int a=0; a<height*width; a++){
        data[a] = 0;
    }
    map.data = data;
    for (int i=0; i<coff.rows(); i++){
        int a = round((obsty[i]*map.info.width + obstx[i])/res);
        map.data[a] = 100;
        int b = round((fronty[i]*map.info.width + frontx[i])/res);
        map.data[b] = -1;
    }

    return map;
}
*/ 

