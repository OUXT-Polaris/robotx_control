#ifndef VRX_CONTROL_VRX_SPEED_CONTROLLER_H_INCLUDED
#define VRX_CONTROL_VRX_SPEED_CONTROLLER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

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
    const ros::NodeHandle nh;
    const ros::NodeHandle pnh;
    ros::Subscriber current_twist_sub_;
    ros::Subscriber target_twist_sub_;
    boost::optional<geometry_msgs::TwistStamped> current_twist_;
    boost::optional<geometry_msgs::TwistStamped> target_twist_;
    void currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    void targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    std::mutex mtx_;
    void publishCurrentCmd();
};

#endif  //VRX_CONTROL_VRX_SPEED_CONTROLLER_H_INCLUDED