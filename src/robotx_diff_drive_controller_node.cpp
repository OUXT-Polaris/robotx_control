// Headers in ros
#include <ros/ros.h>

// Headers in this package
#include <robotx_control/robotx_diff_drive_controller.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotx_diff_drive_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RobotXDiffDriveController controller(nh,pnh);
    controller.run();
    ros::spin();
    return 0;
}