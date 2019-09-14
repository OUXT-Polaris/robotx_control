#ifndef ROBOTX_CONTROL_ROBOTX_DIFF_DRIVE_CONTROLLER_H_INCLUDED
#define ROBOTX_CONTROL_ROBOTX_DIFF_DRIVE_CONTROLLER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <usv_control_msgs/AzimuthThrusterCatamaranDriveStamped.h>
#include <dynamic_reconfigure/server.h>

// Headers in STL
#include <mutex>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>

// Headers in this package
#include <robotx_control/RobotXDiffDriveControllerConfig.h>

class RobotXDiffDriveController
{
public:
    RobotXDiffDriveController(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~RobotXDiffDriveController();
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber current_twist_sub_;
    ros::Subscriber target_twist_sub_;
    ros::Publisher control_command_pub_;
    boost::optional<geometry_msgs::TwistStamped> current_twist_;
    boost::optional<geometry_msgs::TwistStamped> target_twist_;
    void currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    void targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    std::mutex mtx_;
    std::string current_twist_topic_;
    std::string target_twist_topic_;
    std::string control_command_topic_;
    void publishCurrentCmd();
    dynamic_reconfigure::Server<robotx_control::RobotXDiffDriveControllerConfig> dynaparam_server_;
    dynamic_reconfigure::Server<robotx_control::RobotXDiffDriveControllerConfig>::CallbackType dynaparam_callback_func_;
    void dynaparamCallback(robotx_control::RobotXDiffDriveControllerConfig &config, uint32_t level);
    robotx_control::RobotXDiffDriveControllerConfig config_;
    double error_integral_linear_;
    double error_integral_angular_;
};

#endif  //ROBOTX_CONTROL_ROBOTX_DIFF_DRIVE_CONTROLLER_H_INCLUDED