#ifndef VRX_CONTROL_VRX_SPEED_CONTROLLER_H_INCLUDED
#define VRX_CONTROL_VRX_SPEED_CONTROLLER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <usv_control_msgs/AzimuthThrusterCatamaranDriveStamped.h>

// Headers in STL
#include <mutex>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>

class VrxSpeedController
{
public:
    VrxSpeedController(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~VrxSpeedController();
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
};

#endif  //VRX_CONTROL_VRX_SPEED_CONTROLLER_H_INCLUDED