#ifndef AGV_CONTROLLER_HPP
#define AGV_CONTROLLER_HPP
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>


class CONTROL {
    
private:
    // int --;
    bool received;
    float gainLinear;
    float gainAngular;
    float P_Gain;
    float toleranceLinear;
    float toleranceAngular;
    float linearMax;
    float AngularMax;
    float pi;
    int moveCase;
    float acc_linear;
    float acc_angular;
    float velocity_command_frequency;
    float waypoint_num; // 몇 번째 waypoint 인지
    
    cv::Mat_<float> tar;
    cv::Mat_<float> cur;
    cv::Mat_<float> cur_vel;
    cv::Mat_<float> command;
    cv::Mat_<float> waypoints;
    
public:
    CONTROL(float gainLinear_=0.5, float gainAngular_ = 0.3, float toleranceLinear_ = 0.01, float toleranceAngular_ = 0.005, float lin=0.45, float ang=1, float acc_lin=0.7, float acc_ang=2.5, float frequency=50.0) {
        gainLinear = gainLinear_;
        gainAngular = gainAngular_;
        P_Gain = 50000; // 나중에 파라미터로 설정
        toleranceLinear = toleranceLinear_;
        toleranceAngular = toleranceAngular_;
        received = false;
        linearMax = lin;
        AngularMax = ang;
        pi = M_PI;
        moveCase = 0;
        acc_linear = acc_lin;
        acc_angular = acc_ang;
        velocity_command_frequency = frequency;
        waypoint_num = 1;

        tar = cv::Mat_<float>(3, 1);
        cur = cv::Mat_<float>(3, 1);
        cur_vel = cv::Mat_<float>(2, 1);
        command = cv::Mat_<float>(2, 1);
        waypoints = cv::Mat_<float>(2, 4);
        tar = 0.0;
        cur = 0.0; // set to last position, default position, etc.
        command = 0.0;
    }
    
    void setCur(cv::Mat data) { cur = cv::Mat_<float>(data); }
    void setCur_Vel(cv::Mat data) { cur_vel = cv::Mat_<float>(data); } 
    void setTar(cv::Mat data) { tar = cv::Mat_<float>(data); }
    void setReceived(bool data) { received = data; }
    void startMove() { moveCase = 1; }
    void setWaypoint() {
        // 시작 지점 추가 (x: 시작 위치의 x값, y: 시작 위치의 y값)
        waypoints.at<float>(0, 0) = cur.at<float>(0);
        waypoints.at<float>(1, 0) = cur.at<float>(1);
        // 첫번째 웨이포인트 추가 (x: 시작 위치의 x값, y: 0)
        waypoints.at<float>(0, 1) = cur.at<float>(0);
        waypoints.at<float>(1, 1) = 0;

        // 두번째 웨이포인트 추가 (x: 목표 위치의 x값, y: 0)
        waypoints.at<float>(0, 2) = tar.at<float>(0);
        waypoints.at<float>(1, 2) = 0;

        // 세번째 웨이포인트 추가 (x: 목표 위치 x값, y: 목표 위치 y값)
        waypoints.at<float>(0, 3) = tar.at<float>(0);
        waypoints.at<float>(1, 3) = tar.at<float>(1);
        }

    cv::Mat_<float> getCommand() const { return command; }
    float dX(float target) const { return target - cur.at<float>(0); }
    float dY(float target) const { return target - cur.at<float>(1); }
    float dTheta(float target) {
        if (target - cur.at<float>(2) > pi) {
            return (target - cur.at<float>(2)) - 2 * pi;
        }
        else if (target - cur.at<float>(2) <= - pi) {
            return (target - cur.at<float>(2)) + 2 * pi;
        }
        else { return target - cur.at<float>(2); }
    }
    float distance(){
        float curX = cur.at<float>(0);
        float curY = cur.at<float>(1);
        float targetX = waypoints.at<float>(0, waypoint_num);
        float targetY = waypoints.at<float>(1, waypoint_num);
        return sqrt(pow(targetX - curX, 2) + pow(targetY - curY, 2));
    }
    float error(){
        float x1 = waypoints.at<float>(0, waypoint_num-1);
        float y1 = waypoints.at<float>(1, waypoint_num-1);
        float x2 = waypoints.at<float>(0, waypoint_num);
        float y2 = waypoints.at<float>(1, waypoint_num);
        float curX = cur.at<float>(0);
        float curY = cur.at<float>(1);
        std::cout << "x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2 << std::endl;
        std::cout << "curX: " << curX << " curY: " << curY << std::endl;
        // ***************************************************** a, b, c 수정 필요
        float a = (y2 - y1);
        float b = - (x2 - x1);
        float c = - (y2 - y1) * x1 + y1 * (x2 - x1);
        std::cout << "a: " << a << " b: " << b << " c: " << c << std::endl;

        cv::Mat vector1 = (cv::Mat_<float>(3, 1) << curX, curY, 0.0);
        cv::Mat vector2 = (cv::Mat_<float>(3, 1) << x2 - x1, y2 - y1, 0.0);

        cv::Mat result = vector1.cross(vector2);
        if ( result.at<float>(2) >= 0 ) {
            std::cout << "error: " << abs(a * curX + b * curY + c) / sqrt(pow(a, 2) + pow(b, 2)) << std::endl;
            return abs(a * curX + b * curY + c) / sqrt(pow(a, 2) + pow(b, 2));
        }
        else {
            std::cout << "error: " << - abs(a * curX + b * curY + c) / sqrt(pow(a, 2) + pow(b, 2)) << std::endl;
            return -abs(a * curX + b * curY + c) / sqrt(pow(a, 2) + pow(b, 2));
        }
    }

    void moveLinear(){
        
        float targetX = waypoints.at<float>(0, waypoint_num);
        float targetY = waypoints.at<float>(1, waypoint_num);
        command.at<float>(0) = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency + 0.03f, linearMax), gainLinear * distance());
        
        // path에서 많이 벗어날 수록 gainAngular 가 커지도록 설정
        command.at<float>(1) = P_Gain * error();
        std::cout << "command: " << command << std::endl;

        std::cout << "-------------------------------------" << std::endl;
    }
    /* void moveX(float targetX) {
        if (dX(targetX) >= 0) {
            float targetTheta = 0;
            if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;               
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency + 0.03f, linearMax), gainLinear * abs(dX(targetX)));
                }
            }
        
        else {
            float targetTheta = pi;
            if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency + 0.03f, linearMax), gainLinear * abs(dX(targetX)));
            }
        }
    }
    void moveY(float targetY) {
        if (dY(targetY) >= 0) {
            float targetTheta = pi/2;
            if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency + 0.03f, linearMax), gainLinear * abs(dY(targetY)));
            }
        }
        else {
            float targetTheta = -pi/2;
            if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
            else {
                command.at<float>(1) = 0;
                command.at<float>(0) 
                = std::min(std::min(cur_vel.at<float>(0) + acc_linear / velocity_command_frequency + 0.03f, linearMax), gainLinear * abs(dY(targetY)));
            }
        }
    } */
    void moveTheta(float targetTheta) {
        /* if (dTheta(targetTheta) >= 0) {
            // abs를 붙인 이유 : 현재 각속도가 음수일 때 절대값을 취하면서 양수로 바뀌면서 목표 각속도보다 커지는 경우가 생김
            command.at<float>(1) 
            = std::min(std::min(abs(cur_vel.at<float>(1)) + acc_angular / velocity_command_frequency + 0.043f, AngularMax), gainAngular * abs(dTheta(targetTheta)));
        }
        else {
            command.at<float>(1) 
            = -std::min(std::min(abs(cur_vel.at<float>(1)) + acc_angular / velocity_command_frequency + 0.043f, AngularMax), gainAngular * abs(dTheta(targetTheta)));
        } */
        command.at<float>(1) 
            = std::min(AngularMax, gainAngular * dTheta(targetTheta));

    }

    void moveToTarget() {
        switch (moveCase) {
            case 1:{ // waypoints 생성
                setWaypoint();
                std::cout << waypoints << std::endl;

                moveCase = 2;
                break;
            }

            case 2:{ // waypoint를 바라보도록 회전
                float targetTheta;
                if ( dX(waypoints.at<float>(0, waypoint_num)) == 0 && dY(waypoints.at<float>(1, waypoint_num)) > 0 ) {
                    targetTheta = pi/2;
                }
                else if ( dX(waypoints.at<float>(0, waypoint_num)) == 0 && dY(waypoints.at<float>(1, waypoint_num)) < 0 ) {
                    targetTheta = -pi/2;
                }
                else if ( dX(waypoints.at<float>(0, waypoint_num)) > 0 && dY(waypoints.at<float>(1, waypoint_num)) == 0 ) {
                    targetTheta = 0;
                }
                else if ( dX(waypoints.at<float>(0, waypoint_num)) < 0 && dY(waypoints.at<float>(1, waypoint_num)) == 0 ) {
                    targetTheta = pi;
                }
                else if ( dX(waypoints.at<float>(0, waypoint_num)) == 0 && dY(waypoints.at<float>(1, waypoint_num)) == 0 ) {
                    break;
                } 
                else {
                    targetTheta = atan2(waypoints.at<float>(1, waypoint_num) - cur.at<float>(1), waypoints.at<float>(0, waypoint_num) - cur.at<float>(0)); 
                }
                if (abs(dTheta(targetTheta)) >= toleranceAngular) {
                    moveTheta(targetTheta);
                    // std::cout << dTheta(targetTheta) << std::endl;
                }
                else {
                    command.at<float>(1) = 0;
                    command.at<float>(0) = 0;
                    moveCase = 3;
                }
                
                break;

            }

            case 3:{ // waypoint로 이동
                if ( distance() > toleranceLinear ) {
                    moveLinear();
                    
                }
                else {
                    command.at<float>(0) = 0;
                    command.at<float>(1) = 0;
                    if ( waypoint_num < waypoints.cols ) {
                        waypoint_num++;
                        moveCase = 0;
                    }
                    else {
                        moveCase = 0;
                    }
                } 
                break;
            }

        }
        // std::cout << moveCase << std::endl;
    }
};

#endif // AGV_CONTROLLER_HPP


            /* 예전 코드 
            case 1: { // y=0 도착
                if (abs(dY(0)) > toleranceLinear) {
                    moveY(0);            
                }
                else {  
                    command.at<float>(0) = 0;
                    command.at<float>(1) = 0;
                    moveCase = 2;
                }
                break;
            }

            case 2: { // x축으로 회전
                if (dX(targetX) >= 0) {
                    float targetTheta = 0;
                    if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
                    else{
                        command.at<float>(1) = 0;               
                        command.at<float>(0) = 0;       
                        moveCase = 3;                       
                    }
                }
                else {
                    float targetTheta = pi;
                    if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
                    else{
                        command.at<float>(1) = 0;               
                        command.at<float>(0) = 0;
                        moveCase = 3;                              
                    }
                }
                break;
            }

            case 3: { // X축 도착
                if (abs(dX(tar.at<float>(0))) > toleranceLinear) {
                    moveX(tar.at<float>(0));
                }
                else { 
                    command.at<float>(0) = 0;
                    command.at<float>(1) = 0;
                    moveCase = 4;
                }
                break;
            }

            case 4: { // x축으로 회전
                if (dX(targetY) >= 0) {
                    float targetTheta = pi/2;
                    if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
                    else{
                        command.at<float>(1) = 0;               
                        command.at<float>(0) = 0;   
                        moveCase = 5;                           
                    }
                }
                else {
                    float targetTheta = - pi/2;
                    if (abs(dTheta(targetTheta)) >= toleranceAngular) { moveTheta(targetTheta); }
                    else{
                        command.at<float>(1) = 0;               
                        command.at<float>(0) = 0;
                        moveCase = 5;                              
                    }
                }
                break;
            }

            case 5: {
                if (abs(dY(tar.at<float>(1)))  > toleranceLinear)  {
                    moveY(tar.at<float>(1)); 
                }
                else { // Y축 도착
                    command.at<float>(0) = 0; 
                    command.at<float>(1) = 0;
                    moveCase = 0;
                }
                break;
            } */
