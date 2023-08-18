#include <ros/ros.h>
#include "estimator.hpp"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

class EstimatorROSWrapper 
{
private:
    std::unique_ptr<ESTIMATOR> estimator;
    ros::Subscriber detected_position_subscriber_;
    ros::Subscriber detected_velocity_subscriber_;
    ros::Subscriber velocity_command_subscriber_;
    ros::Publisher estimated_position_publisher_;
    ros::Timer estimated_position_timer_;
    int stateSize_;
    int contrSize_;
    float publish_estimated_position_frequency_;
    float estimated_position_x;
    float estimated_position_y;

public:
    EstimatorROSWrapper(ros::NodeHandle *nh) {
        if (!ros::param::get("~stateSize", stateSize_)) { stateSize_ = 6; }
        if (!ros::param::get("~contrSize", contrSize_)) { contrSize_ = 2; }
        if (!ros::param::get("~publish_estimated_position_frequency", publish_estimated_position_frequency_)) { publish_estimated_position_frequency_ = 300.0; }

        estimator.reset(new ESTIMATOR(stateSize_, contrSize_, 1.0 / publish_estimated_position_frequency_));

        detected_position_subscriber_ = nh->subscribe("marker_pos", 1, &EstimatorROSWrapper::callbackPosition, this);
        detected_velocity_subscriber_ = nh->subscribe("cur_vel", 1, &EstimatorROSWrapper::callbackVelocity, this);
        velocity_command_subscriber_ = nh->subscribe("cmd_vel", 1, &EstimatorROSWrapper::callbackCommand, this);
        estimated_position_publisher_ = nh->advertise<geometry_msgs::Pose2D>("estimated_pos", 1);
        estimated_position_timer_ = nh->createTimer(ros::Duration(1.0 / publish_estimated_position_frequency_),&EstimatorROSWrapper::publishEstimatedPosition, this);
    }
    
    void callbackPosition(const geometry_msgs::Pose2D::ConstPtr& msg) {
        if  (msg->x == 0 && msg->y == 0) estimator->setReceived(false); // 마커를 안 보고 있을 때
        else if (estimated_position_x !=  0.0f && estimated_position_y != 0.0f) // estimated position에 값이 있을 때
        {
            if ((abs(msg->x - estimated_position_x) > 0.2) || (abs(msg->y - estimated_position_y) > 0.2)){
                estimator->setReceived(false);
                std::cout << "마커가 튐" << std::endl;
            }
            else {
                estimator->setReceived(true);
                cv::Mat data = (cv::Mat_<float>(3, 1) << msg->x, msg->y, msg->theta);
                estimator->setPosData(data);
            }
        }
        else{ // 처음에 estimated_position에 값이 할당 안되어 있고 마커를 보고 있을 때
            estimator->setReceived(true);
            cv::Mat data = (cv::Mat_<float>(3, 1) << msg->x, msg->y, msg->theta);
            estimator->setPosData(data);
        }
    }

    void callbackVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
        cv::Mat data = (cv::Mat_<float>(2, 1) << msg->linear.x, msg->angular.z);
        estimator->setVelData(data);
    }
    void callbackCommand(const geometry_msgs::Twist::ConstPtr& msg) {
        cv::Mat data = (cv::Mat_<float>(2, 1) << msg->linear.x, msg->angular.z);
        estimator->setCommand(data);
    }
    void publishEstimatedPosition(const ros::TimerEvent &event) {
        estimator->updateControlMatrix();
        estimator->updateMeasurementMatrix();
        cv::Mat_<float> predicted = estimator->predict();
        cv::Mat_<float> corrected = estimator->correct();
    
        geometry_msgs::Pose2D msg;
        msg.x = corrected.at<float>(0);
        msg.y = corrected.at<float>(1);
        estimated_position_x = msg.x;
        estimated_position_y = msg.y;
        float theta = corrected.at<float>(2);
        theta = theta - 2 * M_PI * floor((theta + M_PI) / (2 * M_PI));
        msg.theta = theta;
        estimated_position_publisher_.publish(msg);
        estimator->setReceived(false);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agv_estimator");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    EstimatorROSWrapper estimatorWrapper(&nh);
    ROS_INFO("Estimator is now started");
    
    ros::waitForShutdown();
}
