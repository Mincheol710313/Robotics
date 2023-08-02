#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <fstream>

using namespace std;

ofstream pos_data;

void MarkerPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    pos_data.open("/home/godssi/catkin_ws/src/agv_estimator/src/position_data.txt", ios::app);
    pos_data << "Marker position: " << msg->x << ", " << msg->y << ", " << msg->theta << endl;
}

void CameraPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    pos_data.open("/home/godssi/catkin_ws/src/agv_estimator/src/position_data.txt", ios::app);
    pos_data << "Camera position: " << msg->x << ", " << msg->y << ", " << msg->theta << endl;
}

void EstimatedPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    pos_data.open("/home/godssi/catkin_ws/src/agv_estimator/src/position_data.txt", ios::app);
    pos_data << "Estimated position: " << msg->x << ", " << msg->y << ", " << msg->theta << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "agv_pose_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber marker_pos_sub = nh.subscribe("/marker_pos", 10, MarkerPoseCallback);
    ros::Subscriber camera_pos_sub = nh.subscribe("/camera_pos", 10, CameraPoseCallback);
    ros::Subscriber agv_pos_sub = nh.subscribe("/estimated_pos", 10, EstimatedPoseCallback);

    ros::spin();
}