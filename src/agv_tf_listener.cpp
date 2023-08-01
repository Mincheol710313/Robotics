#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "agv_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;
  ros::Publisher pose_pub_ = node.advertise<geometry_msgs::Pose2D>("/marker_pos", 10);

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("world", "agv_tf", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("world", "agv_tf", ros::Time(0), transform);
      
      // 불러온 pose 값을 geometry_msgs::Pose2D 형태로 변환합니다.
      geometry_msgs::Pose2D estimated_pos; 
      estimated_pos.x = transform.getOrigin().x();
      estimated_pos.y = transform.getOrigin().y();
      estimated_pos.theta = tf::getYaw(transform.getRotation());

      // 변환한 pose 값을 /agv_pose로 publish합니다.
      pose_pub_.publish(estimated_pos);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  return 0;
};