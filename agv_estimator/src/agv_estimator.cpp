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
    float estimated_position_x = 0.0;
    float estimated_position_y = 0.0;

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
        if  (msg->x == 0 && msg->y == 0){
            estimator->setReceived(false);
        }
        else{
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
        float theta = corrected.at<float>(2);
        theta = theta - 2 * M_PI * floor((theta + M_PI) / (2 * M_PI));
        msg.theta = theta;
        std::cout << "estimated_pos 값: " << "(" << msg.x << ", " << msg.y << ")" << std::endl;

        if (estimated_position_x == 0.0 && estimated_position_y == 0.0){
            estimated_position_x = msg.x;
            estimated_position_y = msg.y;
        }
        else if ((abs(msg.x - estimated_position_x) < 0.012) && (abs(msg.y - estimated_position_y) < 0.012)){
            // 이 값은 [(최대 속도) x (publish_estimated_position_frequency_)] 의 5배 정도로 설정해야함
            estimated_position_publisher_.publish(msg);
            estimated_position_x = msg.x;
            estimated_position_y = msg.y;
        }
        else{
            std::cout << "* estimated_pos 값이 튐 *" << std::endl;
        }
        std::cout << "-----------------------" << std::endl;
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
