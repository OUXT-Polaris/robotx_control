// Headers in this package
#include <vrx_control/vrx_speed_controller.h>

VrxSpeedController::VrxSpeedController(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("target_twist_topic", target_twist_topic_, "/target_twist");
    pnh_.param<std::string>("current_twist_topic", current_twist_topic_, "/current_twist");
    pnh_.param<std::string>("control_command_topic", control_command_topic_, "/control_command");
    control_command_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(control_command_topic_,1);
    target_twist_sub_ = nh_.subscribe(target_twist_topic_,1,&VrxSpeedController::targetTwistCallback,this);
    current_twist_sub_ = nh_.subscribe(current_twist_topic_,1,&VrxSpeedController::currentTwistCallback,this);
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