// Headers in ros
#include <ros/ros.h>

// Headers in this package
#include <vrx_control/vrx_speed_controller.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vrx_speed_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    VrxSpeedController controller(nh,pnh);
    controller.run();
    ros::spin();
    return 0;
}