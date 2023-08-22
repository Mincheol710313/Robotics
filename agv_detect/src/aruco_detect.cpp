#include <ros/ros.h>
#include "aruco_detect.hpp"
#include <image_transport/image_transport.h> // image를 subscribe 혹은 publish 할 때 사용
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>

class ArucoDetectorROSWrapper
{
private:
  std::unique_ptr<ARUCO_DETECTOR> aruco_detector_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher camera_pose_pub_;
  ros::Publisher marker_pos_pub_;
  ros::Subscriber camera_pos_sub_;

  float squareLength_;
  float markerLength_;
  string showRejected_;
  string estimatePose_;
  string autoScale_;
  float autoScaleFactor_;
  float markerDistance_;
  bool multipleMarkers_;
  string detectorParams_;
  int refine_;
  int camId_;
  string video_;
  int dictionary_;
  string custom_dictionary_;

public:
  ArucoDetectorROSWrapper(ros::NodeHandle *nh)
      : it_(*nh)
  {
    if (!ros::param::get("~squareLength", squareLength_))
    {
      squareLength_ = 0.062;
    }
    if (!ros::param::get("~markerLength", markerLength_))
    {
      markerLength_ = 0.0496;
    }
    if (!ros::param::get("~showRejected", showRejected_))
    {
      showRejected_ = "false";
    }
    if (!ros::param::get("~estimatePose", estimatePose_))
    {
      estimatePose_ = "/home/godssi/catkin_ws/src/agv_detect/include/calibration_540.yml";
    }
    if (!ros::param::get("~autoScale", autoScale_))
    {
      autoScale_ = "";
    }
    if (!ros::param::get("~markerDistance", markerDistance_))
    {
      markerDistance_ = 1.8;
    }
    if (!ros::param::get("~multipleMarkers", multipleMarkers_))
    {
      multipleMarkers_ = true;
    }
    if (!ros::param::get("~detectorParams", detectorParams_))
    {
      detectorParams_ = "";
    }
    if (!ros::param::get("~refine", refine_))
    {
      refine_ = 2;
    }
    if (!ros::param::get("~camId", camId_))
    {
      camId_ = 4;
    }
    if (!ros::param::get("~video", video_))
    {
      video_ = "";
    }
    if (!ros::param::get("~dictionary", dictionary_))
    {
      dictionary_ = 3;
    }
    if (!ros::param::get("~custom_dictionary", custom_dictionary_))
    {
      custom_dictionary_ = "";
    }

    aruco_detector_.reset(new ARUCO_DETECTOR(squareLength_, markerLength_, showRejected_, estimatePose_, autoScale_, markerDistance_, multipleMarkers_, detectorParams_, refine_, camId_, video_, dictionary_, custom_dictionary_));

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ArucoDetectorROSWrapper::callbakPosition, this);
    // camera_pos_sub_ = nh->subscribe("/camera_pos", 1, &ArucoDetectorROSWrapper::MarkerPositionCallback, this);
    // 변환된 이미지를 publish하는 코드로 생략 가능
    image_pub_ = it_.advertise("/convert_image", 1);
    camera_pose_pub_ = nh->advertise<geometry_msgs::Pose2D>("/camera_pos", 1);
    marker_pos_pub_ = nh->advertise<geometry_msgs::Pose2D>("/marker_pos", 1);
  }

  void callbakPosition(const sensor_msgs::ImageConstPtr &msg)
  {
    const uint8_t *image_data = msg->data.data();

    // 이미지의 높이와 너비 추출
    int image_height = msg->height;
    int image_width = msg->width;

    // 이미지 데이터를 OpenCV의 Mat 형식으로 변환
    Mat image(image_height, image_width, CV_8UC3, const_cast<uint8_t *>(image_data));

    // 받아온 이미지를 통해서 pose를 계산하는 코드
    Vec3d rvec_, tvec_;
    vector<int> markerIds_;
    vector<Vec4i> diamondIds_;
    vector<vector<Point2f>> markerCorners_, rejectedMarkers_, diamondCorners_;
    vector<Vec3d> camPos_;
    Vec3d camPosMean_;
    Mat camRot_;
    double camYaw_;
    geometry_msgs::Pose2D camera_pose_;

    try
    {
      aruco_detector_->setMarkerCornersAndIds(image, markerCorners_, markerIds_);
      aruco_detector_->detectDiamond(image, markerIds_, markerCorners_, diamondCorners_, diamondIds_);
      aruco_detector_->estimateDiamondPose(rvec_, tvec_, diamondCorners_, diamondIds_);
      aruco_detector_->calculateCameraPose(rvec_, tvec_, diamondIds_, camPos_, camRot_, camYaw_);

      if(!camPos_.empty()){
        camera_pose_.x = camPos_.at(0)[0];
        camera_pose_.y = camPos_.at(0)[1];
        camera_pose_.theta = camYaw_;
      }
      else{
        camera_pose_.x = 0;
        camera_pose_.y = 0;
        camera_pose_.theta = 0;
      }
    }
    catch (Exception &e)
    {
      ROS_ERROR("Exception: %s", e.what());
      return;
    }

    // image를 publish하는 코드 생략 가능
    // Mat imageCopy;
    // aruco_detector_->drawResults(image, imageCopy, rvec_, tvec_, markerIds_, markerCorners_, rejectedMarkers_, diamondIds_, diamondCorners_);
    // image_pub_.publish(imageCopy.toImageMsg());
    // camera_pose_pub_.publish(camera_pose_);

    geometry_msgs::Pose2D marker_pose_;
    if (camera_pose_.x == 0 && camera_pose_.y == 0)
    {
      marker_pose_.x = 0;
      marker_pose_.y = 0;
      marker_pose_.theta = 0;
    }
    else
    {
      marker_pose_.x = camera_pose_.x - 0.4 * cos(camera_pose_.theta) + 0.0325 * sin(camera_pose_.theta);
      marker_pose_.y = camera_pose_.y - 0.4 * sin(camera_pose_.theta) - 0.0325 * cos(camera_pose_.theta);
      marker_pose_.theta = camera_pose_.theta;
    }
    marker_pos_pub_.publish(marker_pose_);
  }

  /*
  void MarkerPositionCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
  {
    geometry_msgs::Pose2D marker_pose_;
    if (msg->x == 0 && msg->y == 0)
    {
      marker_pose_.x = 0;
      marker_pose_.y = 0;
      marker_pose_.theta = 0;
    }
    else
    {
      static tf::TransformBroadcaster br;
      tf::Transform camera_transform;
      camera_transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
      tf::Quaternion q;
      q.setRPY(0, 0, msg->theta);
      camera_transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(camera_transform, ros::Time::now(), "world", "camera_tf"));

      static tf::TransformBroadcaster br2;
      tf::Transform agv_transform;
      agv_transform.setOrigin(tf::Vector3(-0.4, -0.0325, 0.0));
      tf::Quaternion q2;
      q2.setRPY(0, 0, 0);
      agv_transform.setRotation(q2);
      br2.sendTransform(tf::StampedTransform(agv_transform, ros::Time::now(), "camera_tf", "agv_tf"));

      tf::StampedTransform transform;
      try
      {
        listener.waitForTransform("world", "agv_tf", ros::Time(0), ros::Duration(0));
        listener.lookupTransform("world", "agv_tf", ros::Time(0), transform);

        marker_pose_.x = transform.getOrigin().x();
        marker_pose_.y = transform.getOrigin().y();
        marker_pose_.theta = tf::getYaw(transform.getRotation());
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }
    marker_pos_pub_.publish(marker_pose_);
  }
  */
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_detect");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ArucoDetectorROSWrapper arucoDetectorWrapper(&nh);
  ROS_INFO("ArucoDetector is now started");

  ros::waitForShutdown();
}