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

        // DetectorParms initialize
        detectorParams = aruco::DetectorParameters::create();
        setDetectParams(detectorParams, refine);

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

    void estimateDiamondPose(vector<Vec3d> &rvecs, vector<Vec3d> &tvecs, vector<vector<Point2f>> &diamondCorners, vector<Vec4i> &diamondIds)
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
                rvecs.push_back(currentRvec[0]);
                tvecs.push_back(currentTvec[0]);
            }
        }
    }

    void calculateCameraPose(vector<Vec3d> &rvecs, vector<Vec3d> &tvecs, vector<Vec4i> &diamondIds, vector<Vec3d> &camPos, Mat &camRot, double &camYaw)
    {
        if (rvecs.empty() || tvecs.empty() || diamondIds.empty())
        {
            return; // Exit the function if any of the required vectors are empty
        }
        for (unsigned int i = 0; i < rvecs.size(); ++i)
        {
            Mat R_ctob, rvec = Mat(rvecs[i]);
            Rodrigues(rvec, R_ctob); // Camera-to-marker rotation matrix

            Mat T_ctob = Mat(tvecs[i]); // Camera-to-marker translation vector

            // Invert transformations
            Mat R_btoc = R_ctob.t();       // Marker-to-camera rotation matrix
            Mat T_btoc = -R_btoc * T_ctob; // Marker-to-camera translation vector

            // Assume marker's absolute position is known and stored in absoluteMarkerPos
            double markerPosX = static_cast<double>((diamondIds[i].val[0] * 32 + diamondIds[i].val[1]) * markerDistance);
            double markerPosY = static_cast<double>((diamondIds[i].val[2] * 32 + diamondIds[i].val[3]) * markerDistance);
            double markerPosZ = 0;
            Mat absoluteMarkerPos = (Mat_<double>(3, 1) << markerPosX, markerPosY, markerPosZ); // Marker's absolute position
            Mat R_m = Mat::eye(3, 3, CV_64F);                                                   // Marker's rotation matrix (identity because it's flat)

            // Compute the absolute position and orientation of the camera
            Mat C = R_m * T_btoc + absoluteMarkerPos; // Camera's absolute position
            Mat R_c = R_m * R_btoc;                   // Camera's rotation matrix

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
    }


    void drawResults(Mat &image, Mat &imageCopy, vector<Vec3d> &rvecs, vector<Vec3d> &tvecs, vector<int> &markerIds, vector<vector<Point2f>> &markerCorners, vector<vector<Point2f>> &rejectedMarkers, vector<Vec4i> &diamondIds, vector<vector<Point2f>> &diamondCorners)
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
                    cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                      squareLength * 1.1f);
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

    Ptr<aruco::DetectorParameters> detectorParams;

    int camId;
    string video;

    Ptr<aruco::Dictionary> dictionary;

    Mat camMatrix;
    Mat distCoeffs;
};

#endif // DETECT_H