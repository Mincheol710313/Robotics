#include <ros/ros.h>
#include "agv_control.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <agv_msgs/Target.h>

class ControlROSWrapper 
{
private:
    std::unique_ptr<CONTROL> control;
    ros::ServiceServer target_position_server_;
    ros::Subscriber estimated_position_subscriber_;
    ros::Subscriber current_velocity_subscriber_;
    ros::Publisher velocity_command_publisher_;
    ros::Timer velocity_command_timer_;

    float gainLinear_;
    float gainAngular_;
    float toleranceLinear_;
    float toleranceAngular_;
    float linearMax_;
    float AngularMax_;
    float publish_velocity_command_frequency_;
    float acc_linear_;
    float acc_angluar_;
    float markerDistance_;

public:
    ControlROSWrapper(ros::NodeHandle *nh) {
        if (!ros::param::get("~gainLinear", gainLinear_)) { gainLinear_ = 0.5; }
        if (!ros::param::get("~gainAngular", gainAngular_)) { gainAngular_ = 1; }
        if (!ros::param::get("~toleranceLinear", toleranceLinear_)) { toleranceLinear_ = 0.01; }
        if (!ros::param::get("~toleranceAngular", toleranceAngular_)) { toleranceAngular_ = 0.005; }
        if (!ros::param::get("~linearMax", linearMax_)) { linearMax_ = 0.45; }
        if (!ros::param::get("~AngularMax", AngularMax_)) { AngularMax_ = 1.0; }
        if (!ros::param::get("~publish_velocity_command_frequency", publish_velocity_command_frequency_)) { publish_velocity_command_frequency_ = 300.0; }
        if (!ros::param::get("~acc_linear", acc_linear_)) { acc_linear_ = 0.7; }
        if (!ros::param::get("~acc_angluar", acc_angluar_)) { acc_angluar_ = 2.5; }
        if (!ros::param::get("~markerDistance", markerDistance_)) { markerDistance_ = 1.8; }

        control.reset(new CONTROL(gainLinear_, gainAngular_, toleranceLinear_, toleranceAngular_, linearMax_, AngularMax_, acc_linear_, acc_angluar_, publish_velocity_command_frequency_));

        estimated_position_subscriber_ = nh->subscribe("estimated_pos", 10, &ControlROSWrapper::callbackEstimatedPosition, this);
        velocity_command_publisher_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        current_velocity_subscriber_= nh->subscribe("cur_vel", 10, &ControlROSWrapper::callbackCurVelocity, this);
        velocity_command_timer_ = nh->createTimer(ros::Duration(1.0 / publish_velocity_command_frequency_),&ControlROSWrapper::publishVelocityCommand, this);
        target_position_server_ = nh->advertiseService("target_pos", &ControlROSWrapper::callbackTargetPosition, this);
        // target_position_subscriber_ = nh->subscribe("target_pos", 10, &ControlROSWrapper::callbackTargetPosition, this);
    }
    bool callbackTargetPosition(agv_msgs::Target::Request &req, agv_msgs::Target::Response &res) { 
        // ros params를 통해서 markerDistance를 받아온 후 이를 이용하여 target position을 설정한다.
        cv::Mat data = (cv::Mat_<float>(3, 1) << markerDistance_* req.pose.x, markerDistance_* req.pose.y, req.pose.theta);
        control->setTar(data);
        control->setReceived(true);
        control->startMove();

        res.success = true;
        res.message = "Received target position.";

        return true;
    }
    void callbackEstimatedPosition(const geometry_msgs::Pose2D::ConstPtr& msg) {
        cv::Mat data = (cv::Mat_<float>(3, 1) << msg->x, msg->y, msg->theta);
        control->setCur(data);
    }
    void callbackCurVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
        cv::Mat data = (cv::Mat_<float>(2, 1) << msg->linear.x, msg->angular.z);
        control->setCur_Vel(data);
    }

    void publishVelocityCommand(const ros::TimerEvent &event) {
        // cv::Mat_<float> target = control->getTar();
        // std::cout << target.at<float>(0) << std::endl;
        // control->moveX(target.at<float>(0));
        control->moveToTarget();

        geometry_msgs::Twist msg;
        cv::Mat_<float> command;
        command = control->getCommand();
        msg.linear.x = command.at<float>(0);
        msg.angular.z = command.at<float>(1);
        velocity_command_publisher_.publish(msg); 
    }

    void stop()
    {
        std::cout << "-----------stop-----------" << std::endl;
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.angular.z = 0;
        velocity_command_publisher_.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agv_control");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ControlROSWrapper controlWrapper(&nh);
    ROS_INFO("controller is now started");

    ros::waitForShutdown();
    controlWrapper.stop();
    
}
