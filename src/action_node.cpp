# include <action_node.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "action_node");
    ROS_INFO("action_node initialized");
    ros::NodeHandle nh("~");
    ActionNode action_node(nh);
    action_node.spin();
    return 0;
}