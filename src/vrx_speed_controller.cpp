// Headers in this package
#include <vrx_control/vrx_speed_controller.h>

VrxSpeedController::VrxSpeedController(ros::NodeHandle nh,ros::NodeHandle pnh):nh(nh),pnh(pnh)
{
    target_twist_sub_ = pnh.subscribe("/target_twist",1,&VrxSpeedController::targetTwistCallback,this);
    current_twist_sub_ = pnh.subscribe("/current_twist",1,&VrxSpeedController::currentTwistCallback,this);
}

VrxSpeedController::~VrxSpeedController()
{

}

void VrxSpeedController::currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    mtx_.lock();
    current_twist_ = *msg;
    mtx_.unlock();
    return;
}

void VrxSpeedController::targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    mtx_.lock();
    target_twist_ = *msg;
    mtx_.unlock();
    return;
}

void VrxSpeedController::run()
{
    boost::thread thread_control(boost::bind(&VrxSpeedController::publishCurrentCmd, this));
}

void VrxSpeedController::publishCurrentCmd()
{
    ros::Rate rate(100);
    while(ros::ok())
    {
        mtx_.lock();
        if(target_twist_ && current_twist_)
        {
            // TODO : impliment speed controller
        }
        mtx_.unlock();
        rate.sleep();
    }
}