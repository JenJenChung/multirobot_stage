# include <action_node.h>

int main(int argc, char **argv){
    // TODO: get the right robot namespace
    ros::init(argc, argv,"action_node");
    ROS_INFO("action_node initialized");
    ros::NodeHandle n;
    ActionNode action_node(n);
    ros::spin();
    return 0;
}