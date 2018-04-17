# include <ros/ros.h>
# include <math.h>
# include <algorithm> 
# include <std_msgs/Int8.h>
# include <nav_msgs/OccupancyGrid.h>
# include <geometry_msgs/Pose2D.h>
# include <geometry_msgs/Twist.h>
# include <Eigen/Dense>
//#, OpenCV?, 

Eigen::MatrixXd map2polar(const nav_msgs::OccupancyGrid &map, geometry_msgs::Pose2D robot, int n_th=4, double thresh=50){
    Eigen::MatrixXd coff = Eigen::MatrixXd::Zero(n_th,3);
    double res = map.info.resolution;
    geometry_msgs::Point origin = map.info.origin.position;
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
    return coff;
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

geometry_msgs::Twist polar2waypoint(const Eigen::MatrixXd &coff, geometry_msgs::Pose2D robot){
    geometry_msgs::Twist out;
    return out;
}

Eigen::MatrixXd addOtherRobots(Eigen::MatrixXd &coff, geoemtry_msgs::Pose2D robot, std::vector<geometry_msgs::Pose2D> other_robots){
    Eigen::MatrixXd out;
    return out;
}

*/