# include <action_node.h>

int main(int argc, char **argv){
    // TODO: get the right robot namespace
    ros::init(argc, argv,"action_node");
    ROS_INFO("action_node initialized");
    ros::NodeHandle nh;
    ActionNode action_node(nh);
    ros::spin();
    return 0;
}