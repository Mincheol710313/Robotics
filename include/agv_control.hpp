#ifndef AGV_CONTROLLER_HPP
#define AGV_CONTROLLER_HPP
#include <opencv2/opencv.hpp>

class CONTROL {
    
private:
    // int --;
    bool received;
    float gainP;
    float distanceTolerance;
    float linearMax;
    float AngularMax;
    float pi;
    int moveCase;
    float acc_linear;
    float acc_angular;
    float velocity_command_frequency;

    cv::Mat_<float> tar;
    cv::Mat_<float> cur;
    cv::Mat_<float> cur_vel;
    cv::Mat_<float> command;
    
public:
    CONTROL(float gain=1, float tolerance=0.01, float lin=0.45, float ang=1, float acc_lin=0.7, float acc_ang=2.5, float frequency=50.0) {
        gainP = gain;
        distanceTolerance = tolerance;
        received = false;
        linearMax = lin;
        AngularMax = ang;
        pi = M_PI;
        moveCase = 0;
        acc_linear = acc_lin;
        acc_angular = acc_ang;
        velocity_command_frequency = frequency;

        tar = cv::Mat_<float>(3, 1);
        cur = cv::Mat_<float>(3, 1);
        cur_vel = cv::Mat_<float>(2, 1);
        command = cv::Mat_<float>(2, 1);
        tar = 0.0;
        cur = 0.0; // set to last position, default position, etc.
        command = 0.0;
    }
    
    void setCur(cv::Mat data) { cur = cv::Mat_<float>(data); }
    void setCur_Vel(cv::Mat data) { cur_vel = cv::Mat_<float>(data);}
    void setTar(cv::Mat data) { tar = cv::Mat_<float>(data); }
    void setReceived(bool data) { received = data; }
    void startMove() { moveCase = 1; }
    cv::Mat_<float> getCommand() const { return command; }
    float dX(float target) const { return target - cur.at<float>(0); }
    float dY(float target) const { return target - cur.at<float>(1); }
    float dTheta(float target) {
        if (target - cur.at<float>(2) >= pi) {
            return (target - cur.at<float>(2)) - 2 * pi;
        }
        else if (target - cur.at<float>(2) <= -pi) {
            return (target - cur.at<float>(2)) + 2 * pi;
        }
        else { return target - cur.at<float>(2); }
    }

    void moveX(float targetX) {
        if (dX(targetX) >= 0) {
            float targetTheta = 0;
            if (abs(dTheta(targetTheta)) >= distanceTolerance/2) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;               
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency, linearMax), gainP * abs(dX(targetX)));
                }
            }
        
        else {
            float targetTheta = pi;
            if (abs(dTheta(targetTheta)) >= distanceTolerance/2) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency, linearMax), gainP * abs(dX(targetX)));
            }
        }
    }
    void moveY(float targetY) {
        if (dY(targetY) >= 0) {
            float targetTheta = pi/2;
            if (abs(dTheta(targetTheta)) >= distanceTolerance/2) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency, linearMax), gainP * abs(dY(targetY)));
            }
        }
        else {
            float targetTheta = -pi/2;
            if (abs(dTheta(targetTheta)) >= distanceTolerance/2) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency, linearMax), gainP * abs(dY(targetY)));
            }
        }
    }
    void moveTheta(float targetTheta) {
        if (dTheta(targetTheta) >= 0) {
            // abs를 붙인 이유 : 현재 각속도가 음수일 때 절대값을 취하면서 양수로 바뀌면서 목표 각속도보다 커지는 경우가 생김
            command.at<float>(1) 
            = std::min(std::min(abs(cur_vel.at<float>(1)) + acc_angular / velocity_command_frequency, AngularMax), gainP * abs(dTheta(targetTheta)));
        }
        else {
            command.at<float>(1) 
            = -std::min(std::min(abs(cur_vel.at<float>(1)) + acc_angular / velocity_command_frequency, AngularMax), gainP * abs(dTheta(targetTheta)));
        }
    }

    void moveToTarget() {
        switch (moveCase) {
            case 1: { 
                if (abs(dY(0)) > distanceTolerance) {
                    moveY(0);            
                }
                else {  // y=0 도착
                    command.at<float>(0) = 0;
                    command.at<float>(1) = 0;
                    moveCase = 2;
                }
                break;
            }
            case 2: {
                if (abs(dX(tar.at<float>(0))) > distanceTolerance) {
                    moveX(tar.at<float>(0));
                }
                else { // X축 도착
                    command.at<float>(0) = 0;
                    command.at<float>(1) = 0;
                    moveCase = 3;
                }
                break;
            }
            case 3: {
                if (abs(dY(tar.at<float>(1)))  > distanceTolerance)  {
                    moveY(tar.at<float>(1)); 
                }
                else { // Y축 도착
                    command.at<float>(0) = 0; 
                    command.at<float>(1) = 0;
                    moveCase = 0;
                }
                break;
            }
        }
        // std::cout << moveCase << std::endl;
    }
};

#endif // AGV_CONTROLLER_HPP
