# include <conversions.h>

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("received map");
    geometry_msgs::Pose2D robot;
    robot.x = 0;
    robot.y = 0;
    robot.theta = 0;
    Eigen::MatrixXd coff = map2polar(*msg, robot);
    ROS_INFO("map converted");
    std::cout << coff << std::endl;
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