#ifndef DETECT_H
#define DETECT_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <cstdlib>
#include "aruco_samples_utility.hpp"
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

namespace
{
    const char *about = "Detect ChArUco markers";
    const char *keys =
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{cd       |       | Input file with custom dictionary }"
        "{c        |       | Output file with calibrated camera parameters }"
        "{as       |       | Automatic scale. The provided number is multiplied by the last"
        "diamond id becoming an indicator of the square length. In this case, the -sl and "
        "-ml are only used to know the relative length relation between squares and markers }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}"
        "{r        |       | show rejected candidates too }"
        "{md       |       | Marker distance (in meters) }"
        "{mm       |       | Multiple marker detection and mean pose estimation }";
}

class ARUCO_DETECTOR
{
public:
    ARUCO_DETECTOR(float sl, float ml, string r, string c, string as, float md, bool mm, string dp, int refine, int ci, string v, int d, string cd)
    {
        squareLength = sl;
        markerLength = ml;
        showRejected = isStringNonEmpty(r);
        estimatePose = c;
        autoScale = isStringNonEmpty(as);
        autoScaleFactor = autoScale ? stof(as) : 1.f;
        markerDistance = md;
        multipleMarkers = mm;
        pastX = 0.0;
        pastY = 0.0;

        // DetectorParms initialize
        detectorParams = aruco::DetectorParameters::create();
        setDetectParams(detectorParams, refine);
        // detectorParams->markerBorderBits = 2;
        // detectorParams->minOtsuStdDev = 2.0;
        // detectorParams->errorCorrectionRate = 0.2;

        // Camera initialize
        camId = ci;
        if (isStringNonEmpty(v))
            video = v;

        // Dictionary initialize
        dictionary = aruco::generateCustomDictionary(32, 3);
        // setDitionary(dictionary, d, cd);

        // CameraParameter Setting
        if (isStringNonEmpty(estimatePose))
        {
            bool readOk = readCameraParameters(estimatePose, camMatrix, distCoeffs);
            if (!readOk)
            {
                cerr << "Invalid camera file" << endl;
                exit(-1);
            }
        }
    }

    bool isStringNonEmpty(const string &str) { return !(str.empty()); }

    void setDetectParams(Ptr<aruco::DetectorParameters> &detectorParams, const int &refine)
    {
        if (refine)
        {
            detectorParams->cornerRefinementMethod = refine;
        }
    }

    void setDitionary(Ptr<aruco::Dictionary> &dictionary, const int &d, const string &cd)
    {
        if (d)
        {
            int dictionaryId = d;
            dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
        }
        else
        {
            cerr << "Dictionary not specified" << endl;
            exit(-1);
        }
    }

    void setInputVideo(VideoCapture &inputVideo, int &waitTime, const string &video, const int &camId)
    {
        if (isStringNonEmpty(video))
        {
            inputVideo.open(video);
            waitTime = 0;
        }
        else
        {
            inputVideo.open(camId);
            waitTime = 10;
        }
    }

    void setMarkerCornersAndIds(const Mat &image, vector<vector<Point2f>> &markerCorners, vector<int> &markerIds)
    {
        aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
    }

    void detectDiamond(const Mat &image, vector<int> &markerIds, vector<vector<Point2f>> &markerCorners, vector<vector<Point2f>> &diamondCorners, vector<Vec4i> &diamondIds)
    {
        if (markerIds.size() > 0)
            aruco::detectCharucoDiamond(image, markerCorners, markerIds, squareLength / markerLength, diamondCorners, diamondIds, camMatrix, distCoeffs);
    }

    void estimateDiamondPose(Vec3d &rvec, Vec3d &tvec, vector<vector<Point2f>> &diamondCorners, vector<Vec4i> &diamondIds)
    {
        if (isStringNonEmpty(estimatePose) && diamondIds.size() > 0)
        {
            for (unsigned int i = 0; i < diamondCorners.size(); i++)
            {
                vector<vector<Point2f>> currentCorners;
                vector<Vec3d> currentRvec, currentTvec;
                currentCorners.push_back(diamondCorners[i]);
                // if autoscale, extract square size from last diamond id
                if (autoScale)
                    float squareLength = autoScaleFactor * float(diamondIds[i].val[3]);
                aruco::estimatePoseSingleMarkers(currentCorners, squareLength, camMatrix,
                                                 distCoeffs, currentRvec, currentTvec);
                rvec = currentRvec[0];
                tvec = currentTvec[0];
            }
        }

        // cout << "rvec: " << rvec << endl;
        // cout << "tvec: " << tvec << endl;
    }

    void calculateCameraPose(Vec3d &rvec, Vec3d &tvec, vector<Vec4i> &diamondIds, vector<Vec3d> &camPos, Mat &camRot, double &camYaw)
    {
        if (diamondIds.empty())
        {
            return; // Exit the function if any of the required vectors are empty
        }

        Mat R_ctob;
        Rodrigues(rvec, R_ctob); // Camera-to-marker rotation matrix : 카메라에서 마커까지의 rotation matrix
        // cout << "R_ctob: " << R_ctob << endl;

        Mat T_ctob = Mat(tvec); // Camera-to-marker translation vector : 카메라에서 마커까지의 translation matrix
        // cout << "T_ctob: " << T_ctob << endl;

        // Invert transformations
        Mat R_btoc = R_ctob.t();       // Marker-to-camera rotation matrix
        Mat T_btoc = -R_btoc * T_ctob; // Marker-to-camera translation vector

        // cout << "R_btoc: " << R_btoc << endl;
        // cout << "T_btoc: " << T_btoc << endl;

        // Assume marker's absolute position is known and stored in absoluteMarkerPos
        curX = static_cast<double>(diamondIds[0].val[0] * 32 + diamondIds[0].val[1]);
        curY = static_cast<double>(diamondIds[0].val[2] * 32 + diamondIds[0].val[3]);

        if (pastX == 0.0 && pastY == 0.0) // 초기값일 때 현재 좌표를 저장
        {
            pastX = curX;
            pastY = curY;
        }
        else if (abs(curX - pastX) <= 1 && abs(curY - pastY) == 0)
        {
            pastX = curX;
            pastY = curY;
        }
        else if (abs(curX - pastX) == 0 && abs(curY - pastY) <= 1)
        {
            pastX = curX;
            pastY = curY;
        }
        else
        {
            curX = 0;
            curY = 0;
        }

        // cout << "저장된 좌표: " << pastX << "," << pastY << endl;
        // cout << "저장된 좌표: " << curX << "," << curY << endl;
        double markerPosX = curX * markerDistance;
        double markerPosY = curY * markerDistance;
        double markerPosZ = 0;

        Mat absoluteMarkerPos = (Mat_<double>(3, 1) << markerPosX, markerPosY, markerPosZ); // Marker's absolute position
        Mat R_m = Mat::eye(3, 3, CV_64F);                                                   // Marker's rotation matrix (identity because it's flat)

        // Compute the absolute position and orientation of the camera
        Mat C = R_m * T_btoc + absoluteMarkerPos; // Camera's absolute position
        Mat R_c = R_m * R_btoc;                   // Camera's rotation matrix

        // z축이 뒤집힌 경우 카메라 포즈를 0으로 초기화
        if (C.at<double>(0, 2) < 0)
        {
            camPos.push_back({0, 0, 0});
            camYaw = 0;
        }
        else
        {
            camPos.push_back({C.at<double>(0, 0), C.at<double>(0, 1), C.at<double>(0, 2)});

            camRot.push_back(R_c);

            tf::Matrix3x3 rotation_matrix(R_c.at<double>(0, 0), R_c.at<double>(0, 1), R_c.at<double>(0, 2),
                                          R_c.at<double>(1, 0), R_c.at<double>(1, 1), R_c.at<double>(1, 2),
                                          R_c.at<double>(2, 0), R_c.at<double>(2, 1), R_c.at<double>(2, 2));

            double yaw_x, yaw_y, pitch, roll;
            rotation_matrix.getEulerYPR(yaw_y, pitch, roll);
            yaw_x = yaw_y + M_PI / 2;
            yaw_x = fmod(yaw_x + M_PI, 2 * M_PI) - M_PI;

            camYaw = yaw_x;
        }
        // printf("camPos: %f, %f, %f\n", camPos[0][0], camPos[0][1], camPos[0][2]);
        // cout << "카메라의 yaw: " << camYaw << endl;
    }

    void drawResults(Mat &image, Mat &imageCopy, Vec3d &rvec, Vec3d &tvec, vector<int> &markerIds, vector<vector<Point2f>> &markerCorners, vector<vector<Point2f>> &rejectedMarkers, vector<Vec4i> &diamondIds, vector<vector<Point2f>> &diamondCorners)
    {
        image.copyTo(imageCopy);
        if (markerIds.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, markerCorners);

        if (diamondIds.size() > 0)
        {
            aruco::drawDetectedDiamonds(imageCopy, diamondCorners, diamondIds);

            if (isStringNonEmpty(estimatePose))
            {
                for (unsigned int i = 0; i < diamondIds.size(); i++)
                    cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvec, tvec,
                                      squareLength * 0.5f);
            }
        }
    }

private:
    float squareLength;
    float markerLength;
    bool showRejected;
    string estimatePose;
    bool autoScale;
    float autoScaleFactor;
    float markerDistance;
    bool multipleMarkers;
    double curX;
    double curY;
    double pastX;
    double pastY;

    Ptr<aruco::DetectorParameters> detectorParams;

    int camId;
    string video;

    Ptr<aruco::Dictionary> dictionary;

    Mat camMatrix;
    Mat distCoeffs;
};

#endif // DETECT_H