#include <ros/ros.h>
#include "agv_control.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <agv_msgs/Target.h>
#include <vector>

using namespace std;

class ControlROSWrapper {

private:
    std::unique_ptr<CONTROL> control;
    ros::ServiceServer target_position_server_;
    ros::Subscriber estimated_position_subscriber_;
    ros::Subscriber current_velocity_subscriber_;
    ros::Publisher velocity_command_publisher_;
    ros::Timer velocity_command_timer_;

    double Kp_Linear_;
    double Ki_Linear_;
    double Kd_Linear_;
    double Kp_Angular_;
    double Ki_Angular_;
    double Kd_Angular_;
    double toleranceLinear_;
    double toleranceAngular_;
    double linearMax_;
    double AngularMax_;
    double publish_velocity_command_frequency_;
    double markerDistance_;
    double lookaheadDistance_;

public:
    ControlROSWrapper(ros::NodeHandle *nh) {
        if (!ros::param::get("~Kp_Linear", Kp_Linear_)) { Kp_Linear_ = 1.7; }
        if (!ros::param::get("~Ki_Angular", Ki_Angular_)) { Ki_Angular_ = 0.0; }
        if (!ros::param::get("~Kd_Linear", Kd_Linear_)) { Kd_Linear_ = 0.0; }
        if (!ros::param::get("~Kp_Angular", Kp_Angular_)) { Kp_Angular_ = 4.7; }
        if (!ros::param::get("~Ki_Angular", Ki_Angular_)) { Ki_Angular_ = 0.0; }
        if (!ros::param::get("~Kd_Angular", Kd_Angular_)) { Kd_Angular_ = 0.0; }
        if (!ros::param::get("~toleranceLinear", toleranceLinear_)) { toleranceLinear_ = 0.01; }
        if (!ros::param::get("~toleranceAngular", toleranceAngular_)) { toleranceAngular_ = 0.005; }
        if (!ros::param::get("~linearMax", linearMax_)) { linearMax_ = 0.45; }
        if (!ros::param::get("~AngularMax", AngularMax_)) { AngularMax_ = 1.0; }
        if (!ros::param::get("~publish_velocity_command_frequency", publish_velocity_command_frequency_)) { publish_velocity_command_frequency_ = 600.0; }
        if (!ros::param::get("~markerDistance", markerDistance_)) { markerDistance_ = 1.8; }
        if (!ros::param::get("~lookaheadDistance", lookaheadDistance_)) { lookaheadDistance_ = 1.0; }

        control.reset(new CONTROL(
            Kp_Linear_, Ki_Linear_, Kd_Linear_, 
            Kp_Angular_, Ki_Angular_, Kd_Angular_, 
            toleranceLinear_, toleranceAngular_, linearMax_, AngularMax_, 
            publish_velocity_command_frequency_, lookaheadDistance_));

        estimated_position_subscriber_ = nh->subscribe("pose", 1, &ControlROSWrapper::callbackCurPose, this);
        velocity_command_publisher_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_velocity_subscriber_= nh->subscribe("cur_vel", 1, &ControlROSWrapper::callbackCurVelocity, this);
        velocity_command_timer_ = nh->createTimer(ros::Duration(1.0 / publish_velocity_command_frequency_),&ControlROSWrapper::publishVelocityCommand, this);
        target_position_server_ = nh->advertiseService("target_pos", &ControlROSWrapper::callbackTargetPosition, this);
    }
    bool callbackTargetPosition(agv_msgs::Target::Request &req, agv_msgs::Target::Response &res) {
        // ros params를 통해서 markerDistance를 받아온 후 이를 이용하여 target position을 설정한다.
        vector<double> data = {markerDistance_* req.pose.x, markerDistance_* req.pose.y, req.pose.theta};
        control->setTar(data);
        control->setReceived(true);
        control->startMove();

        res.success = true;
        res.message = "Received target position.";

        return true;
    }
    void callbackCurPose(const geometry_msgs::Pose2D::ConstPtr& msg) {
        vector<double> data = {msg->x, msg->y, msg->theta};
        control->setCur(data);
    }
    void callbackCurVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
        vector<double> data = {msg->linear.x, msg->angular.z};
        control->setCur_Vel(data);
    }

    void publishVelocityCommand(const ros::TimerEvent &event) {
        control->moveToTarget();
        
        vector<double> command;
        command = control->getCommand();

        geometry_msgs::Twist msg;
        msg.linear.x = command[0];
        msg.angular.z = command[1];

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
