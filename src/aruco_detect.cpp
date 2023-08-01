#include <ros/ros.h>
#include "aruco_detect.hpp"
#include <image_transport/image_transport.h> // image를 subscribe 혹은 publish 할 때 사용
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h> // sensor_image로 들어오는 것을 OpenCV로 처리할 수 있게 image 파일을 바꾸는 용도로 사용 
#include <geometry_msgs/Pose2D.h>

class ArucoDetectorROSWrapper
{
private:
  std::unique_ptr<ARUCO_DETECTOR> aruco_detector_;
  image_transport::ImageTransport it_; 
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pose_pub_;
  ros::Timer estimated_position_timer_;

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
    if (!ros::param::get("~squareLength", squareLength_)) { squareLength_ = 0.062;}
    if (!ros::param::get("~markerLength", markerLength_)) { markerLength_ = 0.0496;}
    if (!ros::param::get("~showRejected", showRejected_)) { showRejected_ = "false";}
    if (!ros::param::get("~estimatePose", estimatePose_)) { estimatePose_ = "/home/godssi/aruco_test/build/calibration_540.yml";}
    if (!ros::param::get("~autoScale", autoScale_)) { autoScale_ = "";}
    if (!ros::param::get("~markerDistance", markerDistance_)) { markerDistance_ = 1;}
    if (!ros::param::get("~multipleMarkers", multipleMarkers_)) { multipleMarkers_ = true;}
    if (!ros::param::get("~detectorParams", detectorParams_)) { detectorParams_ = "";}
    if (!ros::param::get("~refine", refine_)) { refine_ = 1;}
    if (!ros::param::get("~camId", camId_)) { camId_ = 4;}
    if (!ros::param::get("~video", video_)) { video_ = "";}
    if (!ros::param::get("~dictionary", dictionary_)) { dictionary_ = 3;}
    if (!ros::param::get("~custom_dictionary", custom_dictionary_)) { custom_dictionary_ = "";}

    aruco_detector_.reset(new ARUCO_DETECTOR(squareLength_, markerLength_, showRejected_, estimatePose_, autoScale_, markerDistance_, multipleMarkers_, detectorParams_, refine_, camId_, video_, dictionary_, custom_dictionary_));

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ArucoDetectorROSWrapper::callbakPosition, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);
    pose_pub_ = nh->advertise<geometry_msgs::Pose2D>("/camera_pos", 10);
  }

  void callbakPosition(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 받아온 이미지를 통해서 pose를 계산하는 코드
    vector< Vec3d > rvecs_, tvecs_;
    vector< int > markerIds_;
    vector< Vec4i > diamondIds_;
    vector< vector< Point2f > > markerCorners_, rejectedMarkers_, diamondCorners_;
    vector< Vec3d > camPos_;
    Vec3d camPosMean_;
    Mat camRot_;
    double camYaw_;

    aruco_detector_->setMarkerCornersAndIds(cv_ptr->image, markerCorners_, markerIds_, rejectedMarkers_);
    aruco_detector_->detectDiamond(cv_ptr->image, markerIds_, markerCorners_, diamondCorners_, diamondIds_);
    aruco_detector_->estimateDiamondPose(rvecs_, tvecs_, diamondCorners_, diamondIds_);
    aruco_detector_->calculateCameraPose(rvecs_, tvecs_, diamondIds_, camPos_, camRot_, camYaw_);
    aruco_detector_->calculateMeanPose(camPos_, camPosMean_);

    geometry_msgs::Pose2D camera_pose_;
    camera_pose_.x = camPos_.at<double>(0,0);
    camera_pose_.y = camPos_.at<double>(0,1);
    camera_pose_.theta = camYaw_;

    // image 생성하는 코드
    Mat imageCopy;
    aruco_detector_->drawResults(cv_ptr->image, imageCopy, rvecs_, tvecs_, markerIds_, markerCorners_, rejectedMarkers_, diamondIds_, diamondCorners_);
    image_pub_.publish(cv_ptr->toImageMsg());
    pose_pub_.publish(camera_pose_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_detect");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ArucoDetectorROSWrapper arucoDetectorWrapper(&nh);
  ROS_INFO("ArucoDetector is now started");

  ros::waitForShutdown();
}