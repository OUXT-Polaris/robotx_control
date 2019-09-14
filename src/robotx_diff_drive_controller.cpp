// Headers in this package
#include <robotx_control/robotx_diff_drive_controller.h>

RobotXDiffDriveController::RobotXDiffDriveController(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("target_twist_topic", target_twist_topic_, "/target_twist");
    pnh_.param<std::string>("current_twist_topic", current_twist_topic_, "/current_twist");
    pnh_.param<std::string>("control_command_topic", control_command_topic_, "/control_command");
    dynaparam_callback_func_ = boost::bind(&RobotXDiffDriveController::dynaparamCallback, this, _1, _2);
    dynaparam_server_.setCallback(dynaparam_callback_func_);
    control_command_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(control_command_topic_,1);
    target_twist_sub_ = nh_.subscribe(target_twist_topic_,1,&RobotXDiffDriveController::targetTwistCallback,this);
    current_twist_sub_ = nh_.subscribe(current_twist_topic_,1,&RobotXDiffDriveController::currentTwistCallback,this);
}

RobotXDiffDriveController::~RobotXDiffDriveController()
{

}

void RobotXDiffDriveController::currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    mtx_.lock();
    current_twist_ = *msg;
    mtx_.unlock();
    return;
}

void RobotXDiffDriveController::targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    mtx_.lock();
    target_twist_ = *msg;
    mtx_.unlock();
    return;
}

void RobotXDiffDriveController::dynaparamCallback(robotx_control::RobotXDiffDriveControllerConfig &config, uint32_t level)
{
    mtx_.lock();
    config_ = config;
    mtx_.unlock();
    return;
}

void RobotXDiffDriveController::run()
{
    boost::thread thread_control(boost::bind(&RobotXDiffDriveController::publishCurrentCmd, this));
}

void RobotXDiffDriveController::publishCurrentCmd()
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