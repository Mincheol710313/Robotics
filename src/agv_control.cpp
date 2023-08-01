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

    float gain_;
    float tolerance_;
    float linearMax_;
    float AngularMax_;
    float publish_velocity_command_frequency_;
    float acc_linear_;
    float acc_angluar_;
    float marker_distance_;

public:
    ControlROSWrapper(ros::NodeHandle *nh) {
        if (!ros::param::get("~gain", gain_)) { gain_ = 1; }
        if (!ros::param::get("~tolerance", tolerance_)) { tolerance_ = 0.01; }
        if (!ros::param::get("~linearMax", linearMax_)) { linearMax_ = 0.45; }
        if (!ros::param::get("~AngularMax", AngularMax_)) { AngularMax_ = 1.0; }
        if (!ros::param::get("~publish_velocity_command_frequency", publish_velocity_command_frequency_)) { publish_velocity_command_frequency_ = 50.0; }
        if (!ros::param::get("~acc_linear", acc_linear_)) { acc_linear_ = 0.7; }
        if (!ros::param::get("~acc_angluar", acc_angluar_)) { acc_angluar_ = 2.5; }
        if (!ros::param::get("~marker_distance", marker_distance_)) { marker_distance_ = 1.8; }

        control.reset(new CONTROL(gain_, tolerance_, linearMax_, AngularMax_, acc_linear_, acc_angluar_, publish_velocity_command_frequency_));

        estimated_position_subscriber_ = nh->subscribe("estimated_pos", 10, &ControlROSWrapper::callbackEstimatedPosition, this);
        velocity_command_publisher_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        current_velocity_subscriber_= nh->subscribe("cur_vel", 10, &ControlROSWrapper::callbackCurVelocity, this);
        velocity_command_timer_ = nh->createTimer(ros::Duration(1.0 / publish_velocity_command_frequency_),&ControlROSWrapper::publishVelocityCommand, this);
        target_position_server_ = nh->advertiseService("target_pos", &ControlROSWrapper::callbackTargetPosition, this);
        // target_position_subscriber_ = nh->subscribe("target_pos", 10, &ControlROSWrapper::callbackTargetPosition, this);
    }
    bool callbackTargetPosition(agv_msgs::Target::Request &req, agv_msgs::Target::Response &res) { 
        cv::Mat data = (cv::Mat_<float>(3, 1) << marker_distance_ * req.pose.x, marker_distance_ * req.pose.y, req.pose.theta);
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
}