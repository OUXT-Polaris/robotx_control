// Headers in this package
#include <robotx_control/robotx_diff_drive_controller.h>

RobotXDiffDriveController::RobotXDiffDriveController(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    error_integral_linear_velocity_ = 0.0;
    error_integral_angular_velocity_ = 0.0;
    pnh_.param<std::string>("target_twist_topic", target_twist_topic_, "/target_twist");
    pnh_.param<std::string>("current_twist_topic", current_twist_topic_, "/current_twist");
    pnh_.param<std::string>("control_command_topic", control_command_topic_, "/control_command");
    pnh_.param<std::string>("reset_command_topic", reset_command_topic_, "/reset_command");
    dynaparam_callback_func_ = boost::bind(&RobotXDiffDriveController::dynaparamCallback, this, _1, _2);
    dynaparam_server_.setCallback(dynaparam_callback_func_);
    control_command_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(control_command_topic_,1);
    target_twist_sub_ = nh_.subscribe(target_twist_topic_,1,&RobotXDiffDriveController::targetTwistCallback,this);
    current_twist_sub_ = nh_.subscribe(current_twist_topic_,1,&RobotXDiffDriveController::currentTwistCallback,this);
    reset_command_sub_ =  nh_.subscribe(reset_command_topic_,1,&RobotXDiffDriveController::resetCommandCallcack,this);
}

RobotXDiffDriveController::~RobotXDiffDriveController()
{

}

void RobotXDiffDriveController::resetCommandCallcack(const std_msgs::Empty::ConstPtr msg)
{
    mtx_.lock();
    error_integral_linear_velocity_ = 0.0;
    error_integral_angular_velocity_ = 0.0;
    mtx_.unlock();
    return;
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
            double error_linear_velocity = target_twist_->twist.linear.x - current_twist_->twist.linear.x;
            double error_angular_velocity =  target_twist_->twist.angular.z - current_twist_->twist.angular.z;
            error_integral_linear_velocity_ = error_integral_linear_velocity_ + error_linear_velocity;
            error_integral_angular_velocity_ = error_integral_angular_velocity_ + error_angular_velocity;

            // PI Control
            double control_linear_velocity = config_.kp_linear_velocity*error_linear_velocity 
                + config_.ki_linear_velocity*error_integral_linear_velocity_;
            double control_angular_velocity = config_.kp_angular_velocity*error_angular_velocity 
                + config_.ki_angular_velocity*error_integral_angular_velocity_;

            // Calculate Thrust
            double control_left_thrust = control_linear_velocity - (config_.tread/2.0)*control_angular_velocity;
            double control_right_thrust = control_linear_velocity + (config_.tread/2.0)*control_angular_velocity;
            control_left_thrust = control_left_thrust/config_.left_thruster_coefficient;
            control_right_thrust = control_right_thrust/config_.right_thruster_coefficient;

            if(control_left_thrust>1.0)
            {
                control_left_thrust = 1.0;
            }
            else if(control_left_thrust<-1.0)
            {
                control_left_thrust = -1.0;
            }
            if(control_right_thrust>1.0)
            {
                control_right_thrust = 1.0;
            }
            else if(control_right_thrust<-1.0)
            {
                control_right_thrust = -1.0;
            }

            // Publish Command
            usv_control_msgs::AzimuthThrusterCatamaranDriveStamped msg;
            msg.header = current_twist_->header;
            msg.command.left_thrust_cmd = control_left_thrust;
            msg.command.right_thrust_cmd = control_right_thrust;
            control_command_pub_.publish(msg);
        }
        mtx_.unlock();
        rate.sleep();
    }
}