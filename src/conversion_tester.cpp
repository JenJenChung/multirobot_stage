# include <conversions.h>
# include <ros/ros.h>

class ConversionTester{
    public:
        ConversionTester(ros::NodeHandle, std::string, std::string);
        ~ConversionTester() {}

    private:
        ros::NodeHandle nh_;

        ros::Subscriber odom_sub_;
        ros::Subscriber map_sub_;

        nav_msgs::OccupancyGrid map_;
        nav_msgs::Odometry odom_;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&);
};

ConversionTester::ConversionTester(ros::NodeHandle nh, std::string map_topic, std::string odom_topic){
    nh_ = nh;

    ros::Subscriber map_sub_ = nh_.subscribe(map_topic, 1, &ConversionTester::mapCallback, this);
    ROS_INFO("subscribed to map topic");

    ros::Subscriber odom_sub_ = nh_.subscribe(odom_topic, 1, &ConversionTester::odomCallback, this);
    ROS_INFO("subscribed to odom topic");

}

void ConversionTester::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("received map");
    Eigen::MatrixXd coff = mapToPolar(*msg, &odom_, 0, 1);
    ROS_INFO("map converted");
    std::cout << coff << std::endl;
}

void ConversionTester::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_ = *msg;
}

int main(int argc, char **argv){
    std::string map_topic, odom_topic;
    if (argc>1){
        map_topic = argv[1];
        odom_topic = argv[2];
    } else {
        map_topic = "robot_0/map";
        odom_topic = "robot_0/odom";
    }
    ros::init(argc, argv, "conversion_tester");
    ROS_INFO("conversion_tester initialized");
    ros::NodeHandle nh;
    ConversionTester tester(nh, map_topic, odom_topic);
    ros::spin();
    return 0;
}