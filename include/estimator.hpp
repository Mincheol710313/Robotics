#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

#include <opencv2/opencv.hpp>

class ESTIMATOR {
public:
    ESTIMATOR(int state=6, int control=2, float dt_=0.1) {
        stateSize = state;
        measSize_marker = 3;
        measSize_encoder = 2;
        contrSize = control;
        measSize_fusion = measSize_marker + measSize_encoder;
        dt = dt_;
        received = false;

        posData = cv::Mat_<float>(3, 1);
        velData = cv::Mat_<float>(2, 1);
        command = cv::Mat_<float>(2, 1);
        posData = 0.0; // set to last pos/ default pos/ etc.
        velData = 0.0;
        command = 0.0;

        kf.init(stateSize, measSize_fusion, contrSize, CV_32F);
        // Transition Matrix F                                  state: [x, y, θ,xd,yd, w]
        kf.transitionMatrix = (cv::Mat_<float>(stateSize, stateSize) << 1, 0, 0,dt, 0, 0,  
                                                                        0, 1, 0, 0,dt, 0,  
                                                                        0, 0, 1, 0, 0,dt,  
                                                                        0, 0, 0, 0, 0, 0,  
                                                                        0, 0, 0, 0, 0, 0,  
                                                                        0, 0, 0, 0, 0, 0);
        // Measurement Matirx H(Observation Matrix)             measurement: [x, y, θ, v, w]
        kf.measurementMatrix = (cv::Mat_<float>(measSize_fusion, stateSize) <<  1, 0, 0, 0, 0, 0,   // x
                                                                                0, 1, 0, 0, 0, 0,   // y
                                                                                0, 0, 1, 0, 0, 0,   // θ
                                                                                0, 0, 0, 0, 0, 0,   // v
                                                                                0, 0, 0, 0, 0, 1);  // w
        // Control Matrix B
        kf.controlMatrix = (cv::Mat_<float>(stateSize, contrSize) <<0, 0,  // x
                                                                    0, 0,  // y
                                                                    0, 0,  // θ
                                                                    0, 0,  // xd
                                                                    0, 0,  // yd
                                                                    0, 1); // w
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));     // Q
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1)); // R
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));           // P
    }
    
    void setPosData(cv::Mat data) { posData = cv::Mat_<float>(data); }
    void setCameraPosData(cv::Mat data) { cameraPosData = cv::Mat_<float>(data); }
    cv::Mat_<float> getCameraPosData() const { return cameraPosData; }
    void setVelData(cv::Mat data) { velData = cv::Mat_<float>(data); }
    void setCommand(cv::Mat data) { command = cv::Mat_<float>(data); }
    void setReceived(bool data) { received = data; }

    void updateControlMatrix() {
        if (received) {
            kf.controlMatrix.at<float>(3,0) = cos(posData.at<float>(2));
            kf.controlMatrix.at<float>(4,0) = sin(posData.at<float>(2));
        }
        else {
            kf.controlMatrix.at<float>(3,0) = cos(kf.statePost.at<float>(2));
            kf.controlMatrix.at<float>(4,0) = sin(kf.statePost.at<float>(2));
        }
    }
    void updateMeasurementMatrix() {
        if (received) {
            kf.measurementMatrix.at<float>(3,3) = cos(posData.at<float>(2));
            kf.measurementMatrix.at<float>(3,4) = sin(posData.at<float>(2));
        }
        else {
            kf.measurementMatrix.at<float>(3,3) = cos(kf.statePost.at<float>(2));
            kf.measurementMatrix.at<float>(3,4) = sin(kf.statePost.at<float>(2));
        }
    }
    // 예측 단계 : 사용자의 입력(command)을 가 했을 때 예상되는 측정값(statePre) 계산
    cv::Mat_<float> predict() {
        kf.predict(command);
        return kf.statePre;
    }

    // 보정 단계 : 앞서 예측된 측정값(statePre)과 실제 측정값(sensor)을 비교하여 보정
    cv::Mat_<float> correct() {
        cv::Mat sensor_marker;
        if (received) sensor_marker = posData.clone();
        else kf.statePost.rowRange(0,3).copyTo(sensor_marker);
        cv::Mat sensor_encoder = velData.clone();
        cv::Mat sensor_fusion;
        cv::vconcat(sensor_marker, sensor_encoder, sensor_fusion);

        kf.correct(sensor_fusion);
        return kf.statePost;
    }

private:
    int stateSize;
    int measSize_marker;
    int measSize_encoder;
    int measSize_fusion;
    int contrSize;
    float dt;
    bool received;

    cv::Mat_<float> cameraPosData;
    cv::Mat_<float> posData;
    cv::Mat_<float> velData;
    cv::Mat_<float> command;
    cv::KalmanFilter kf;
};

#endif // ESTIMATOR_HPP