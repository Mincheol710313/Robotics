#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>

class AGV_TF_LISTENER {
private:
  tf::TransformListener listener;
  ros::Subscriber filter_camera_pose_sub_;
  ros::Publisher marker_pos_pub_;

  geometry_msgs::Pose2D filter_camera_pos;
  geometry_msgs::Pose2D estimated_pos;

  ros::Timer marker_pos_timer_;
  float publish_marker_pos_frequency_;

public:
  AGV_TF_LISTENER(ros::NodeHandle *nh) {
    if (!ros::param::get("~publish_estimated_position_frequency", publish_marker_pos_frequency_)) { publish_marker_pos_frequency_ = 60.0; }
    
    filter_camera_pose_sub_ = nh->subscribe("filter_camera_pos", 10, &AGV_TF_LISTENER::CameraPoseCallback, this);
    marker_pos_pub_ = nh->advertise<geometry_msgs::Pose2D>("/marker_pos", 10);
    marker_pos_timer_ = nh->createTimer(ros::Duration(1.0 / publish_marker_pos_frequency_),&AGV_TF_LISTENER::publishMarkerPos, this);
  
  }

  void CameraPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    filter_camera_pos.x = msg->x;
    filter_camera_pos.y = msg->y;
    filter_camera_pos.theta = msg->theta;
  }

  void publishMarkerPos(const ros::TimerEvent &event) {
    tf::StampedTransform transform;
    try{
      if(filter_camera_pos.x == 0 && filter_camera_pos.y == 0){
        geometry_msgs::Pose2D marker_pos;
        marker_pos.x = 0;
        marker_pos.y = 0;
        marker_pos.theta = 0;

        marker_pos_pub_.publish(marker_pos);
      }
      else{
        listener.waitForTransform("world", "agv_tf", ros::Time(0), ros::Duration(0));
        listener.lookupTransform("world", "agv_tf", ros::Time(0), transform);
      
        // 불러온 pose 값을 geometry_msgs::Pose2D 형태로 변환합니다.
        geometry_msgs::Pose2D marker_pos; 
        marker_pos.x = transform.getOrigin().x();
        marker_pos.y = transform.getOrigin().y();
        marker_pos.theta = tf::getYaw(transform.getRotation());

        // 변환한 pose 값을 /agv_pose로 publish합니다.
        marker_pos_pub_.publish(marker_pos);
      }
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "agv_tf_listener");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  AGV_TF_LISTENER agv_tf_listener(&nh);
  ROS_INFO("agv_tf_listener node has started.");
  
  ros::waitForShutdown();
};