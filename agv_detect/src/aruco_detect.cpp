#include <ros/ros.h>
#include "aruco_detect.hpp"
#include <image_transport/image_transport.h> // image를 subscribe 혹은 publish 할 때 사용
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h> // sensor_image로 들어오는 것을 OpenCV로 처리할 수 있게 image 파일을 바꾸는 용도로 사용
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
  tf::Transform agv_transform_;
  ros::Publisher camera_pose_pub_;
  ros::Publisher marker_pos_pub_;
  ros::Subscriber camera_pos_sub_;
  tf::TransformListener listener;

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
    if (!ros::param::get("~estimatePose", estimatePose_)) { estimatePose_ = "/home/godssi/catkin_ws/src/agv_detect/include/calibration_540.yml";}
    if (!ros::param::get("~autoScale", autoScale_)) { autoScale_ = "";}
    if (!ros::param::get("~markerDistance", markerDistance_)) { markerDistance_ = 1.8;}
    if (!ros::param::get("~multipleMarkers", multipleMarkers_)) { multipleMarkers_ = true;}
    if (!ros::param::get("~detectorParams", detectorParams_)) { detectorParams_ = "";}
    if (!ros::param::get("~refine", refine_)) { refine_ = 2;}
    if (!ros::param::get("~camId", camId_)) { camId_ = 4;}
    if (!ros::param::get("~video", video_)) { video_ = "";}
    if (!ros::param::get("~dictionary", dictionary_)) { dictionary_ = 3;}
    if (!ros::param::get("~custom_dictionary", custom_dictionary_)) { custom_dictionary_ = "";}

    aruco_detector_.reset(new ARUCO_DETECTOR(squareLength_, markerLength_, showRejected_, estimatePose_, autoScale_, markerDistance_, multipleMarkers_, detectorParams_, refine_, camId_, video_, dictionary_, custom_dictionary_));

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ArucoDetectorROSWrapper::callbakPosition, this);
    // 변환된 이미지를 publish하는 코드로 생략 가능
    image_pub_ = it_.advertise("/convert_image", 10);
    camera_pose_pub_ = nh->advertise<geometry_msgs::Pose2D>("/camera_pos", 10);
    marker_pos_pub_ = nh->advertise<geometry_msgs::Pose2D>("/marker_pos", 10);
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

    aruco_detector_->setMarkerCornersAndIds(cv_ptr->image, markerCorners_, markerIds_);
    aruco_detector_->detectDiamond(cv_ptr->image, markerIds_, markerCorners_, diamondCorners_, diamondIds_);
    aruco_detector_->estimateDiamondPose(rvecs_, tvecs_, diamondCorners_, diamondIds_);
    aruco_detector_->calculateCameraPose(rvecs_, tvecs_, diamondIds_, camPos_, camRot_, camYaw_);
    aruco_detector_->calculateMeanPose(camPos_, camPosMean_);

    geometry_msgs::Pose2D camera_pose_;
    camera_pose_.x = camPosMean_[0];
    camera_pose_.y = camPosMean_[1];
    camera_pose_.theta = camYaw_;

    // image를 publish하는 코드 생략 가능
    Mat imageCopy;
    aruco_detector_->drawResults(cv_ptr->image, imageCopy, rvecs_, tvecs_, markerIds_, markerCorners_, rejectedMarkers_, diamondIds_, diamondCorners_);
    image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCopy).toImageMsg());
    camera_pose_pub_.publish(camera_pose_);

    // marker_pos를 publish하는 코드(tf를 이용하지 않고 pose를 publish하는 코드)
    // M_PI와 -M_PI를 구분해야 하는 코드가 위에서 구현되어야 한다.-> -M_PI와 M_PI를 같은 각도로 인식해야함.
    // geometry_msgs::Pose2D marker_pos;
    // if(camera_pose_.theta >= 0 && camera_pose_.theta < M_PI/2){
    //   marker_pos.x = camera_pose_.x - 0.4;
    //   marker_pos.y = camera_pose_.y - 0.0325;
    // }
    // else if(camera_pose_.theta >= M_PI/2 && camera_pose_.theta < M_PI){
    //   marker_pos.x = camera_pose_.x + 0.0325;
    //   marker_pos.y = camera_pose_.y - 0.4;;
    // }
    // else if(camera_pose_.theta = M_PI){
    //   marker_pos.x = camera_pose_.x + 0.4;
    //   marker_pos.y = camera_pose_.y + 0.0325;
    // }
    // else if(camera_pose_.theta < 0 && camera_pose_.theta >= -M_PI/2){
    //   marker_pos.x = camera_pose_.x + 0.0325 ;
    //   marker_pos.y = camera_pose_.y + 0.4;
    // }
    // else{
    //   marker_pos.x = camera_pose_.x + 0.0325;
    //   marker_pos.y = camera_pose_.y + 0.4;
    // }
    // marker_pos.theta = camera_pose_.theta;

    // marker_pos_pub_.publish(marker_pos);
    // marker_pos_pub_.publish(marker_pos);
    
    // // camera_tf를 publish하는 코드
    static tf::TransformBroadcaster br;
    tf::Transform camera_transform_;
    camera_transform_.setOrigin( tf::Vector3(camera_pose_.x, camera_pose_.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, camera_pose_.theta);
    camera_transform_.setRotation(q);
    br.sendTransform(tf::StampedTransform(camera_transform_, ros::Time::now(), "world", "camera_tf"));

    // agv_tf를 publish하는 코드
    static tf::TransformBroadcaster br2;
    tf::Transform agv_transform_;
    agv_transform_.setOrigin( tf::Vector3(-0.4, -0.0325, 0.0) );
    tf::Quaternion q2;
    q2.setRPY(0, 0, 0);
    agv_transform_.setRotation(q2);
    br2.sendTransform(tf::StampedTransform(agv_transform_, ros::Time::now(), "camera_tf", "agv_tf"));

    // marker_pos를 publish하는 코드
    tf::StampedTransform transform;
    try{
       if(camera_pose_.x == 0 && camera_pose_.y == 0){
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

        if(marker_pos.x <= -0.4 || marker_pos.x <= -0.0325 || marker_pos.y <= -0.4 || marker_pos.y <= -0.0325){
          marker_pos.x = 0;
          marker_pos.y = 0;
          marker_pos.theta = 0;
        }

        // 변환한 pose 값을 /agv_pose로 publish합니다.
        marker_pos_pub_.publish(marker_pos);
      }
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_detect");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ArucoDetectorROSWrapper arucoDetectorWrapper(&nh);
  ROS_INFO("ArucoDetector is now started");

  ros::waitForShutdown();
}

