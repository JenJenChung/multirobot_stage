# include <ros/ros.h>
# include <math.h>
# include <algorithm> 
# include <std_msgs/Int8.h>
# include <nav_msgs/OccupancyGrid.h>
# include <geometry_msgs/Pose2D.h>
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
            int x = round(sin(robot.theta + theta)*l + (robot.x-origin.x)/res);
            int y = round(cos(robot.theta + theta)*l + (robot.y-origin.y)/res);
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

nav_msgs::OccupancyGrid polar2map(const Eigen::MatrixXd &coff, geometry_msgs::Pose2D robot, double res=0.005){
    nav_msgs::OccupancyGrid map;
    map.info.resolution = res;
    
    std::vector<double> obstx, obsty, frontx, fronty;
    for (int i=0; i<coff.rows(); i++){
        obstx.push_back(sin(robot.theta + coff(i,0))*coff(i,1) + robot.x);
        obsty.push_back(cos(robot.theta + coff(i,0))*coff(i,1) + robot.y);
        frontx.push_back(sin(robot.theta + coff(i,0))*coff(i,2) + robot.x);
        fronty.push_back(cos(robot.theta + coff(i,0))*coff(i,2) + robot.y);
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
        int a = round((obstx[i]*map.info.width + obsty[i])/res);
        map.data[a] = 100;
        int b = round((frontx[i]*map.info.width + fronty[i])/res);
        map.data[b] = -1;
    }

    return map;
}

void callback(const nav_msgs::OccupancyGrid& msg){
    ROS_INFO("received map");
    geometry_msgs::Pose2D robot;
    robot.x = 0;
    robot.y = 0;
    robot.theta = 0;
    Eigen::MatrixXd coff = map2polar(msg, robot);
    ROS_INFO("map converted");
    std::cout << coff << std::endl;
    nav_msgs::OccupancyGrid map = polar2map(coff, robot);
    std::cout << map << std::endl;
}

int main(int argc, char **argv){
    std::string topic = argv[1];
    ros::init(argc, argv, "listener");
    ROS_INFO("listener initialized");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(topic, 1, callback);
    ROS_INFO("subscribed to topic");
    ros::spin();
    return 0;
}