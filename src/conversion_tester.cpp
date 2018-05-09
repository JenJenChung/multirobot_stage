# include <multirobot_stage/conversions.h>
# include <ros/ros.h>

class ConversionTester{
    public:
        ConversionTester(ros::NodeHandle, std::string, std::string);
        ~ConversionTester() {}

    private:
        ros::NodeHandle nh_;

        ros::Subscriber odom_sub_;
        ros::Subscriber map_sub_;

        Eigen::MatrixXd state_;
        nav_msgs::Odometry odom_;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&);
};

ConversionTester::ConversionTester(ros::NodeHandle nh, std::string map_topic, std::string odom_topic){
    nh_ = nh;

    map_sub_ = nh_.subscribe(map_topic, 10, &ConversionTester::mapCallback, this);
    ROS_INFO_STREAM("subscribed to " << map_topic);

    odom_sub_ = nh_.subscribe(odom_topic, 10, &ConversionTester::odomCallback, this);
    ROS_INFO_STREAM("subscribed to " << odom_topic);

}

void ConversionTester::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    state_ = mapToPolar(*msg, &odom_, 0, 1);
    std::cout << state_ << std::endl;
}

void ConversionTester::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_ = *msg;
    ROS_INFO_STREAM("Robot Odometry:\n" << odom_.pose.pose);
}

int main(int argc, char **argv){
    std::string map_topic, odom_topic;
    if (argc>1){
        map_topic = argv[1];
        odom_topic = argv[2];
    } else {
        map_topic = "/robot_0/map";
        odom_topic = "/robot_0/odom";
    }
    ros::init(argc, argv, "conversion_tester");
    ROS_INFO("conversion_tester initialized");
    ros::NodeHandle nh;
    ConversionTester tester(nh, map_topic, odom_topic);
    ros::spin();
    return 0;
}