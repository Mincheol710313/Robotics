#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_tf"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/filter_camera_pos", 10, &poseCallback);

  ros::spin();
  return 0;
};