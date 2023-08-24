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
    float Kp_Linear;
    float Ki_Linear;
    float Kd_Linear;
    float Kp_Angular;
    float Ki_Angular;
    float Kd_Angular;
    /* float P_Gain;
    float I_Gain;
    float D_Gain; */
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
    float past_distance; // 이전 거리, D 제어를 하기 위함
    float sum_distance; // 거리의 합, I 제어를 하기 위함
    float past_dTheta; // 이전 각도, D 제어를 하기 위함
    float sum_dTheta; // 각도의 합, I 제어를 하기 위함
    float past_error; // 이전 에러, D 제어를 하기 위함
    float sum_error; // 에러의 합, I 제어를 하기 위함
    float targetTheta;
    
    cv::Mat_<float> tar;
    cv::Mat_<float> cur;
    cv::Mat_<float> cur_vel;
    cv::Mat_<float> command;
    cv::Mat_<float> waypoints;
    
public:
    CONTROL(float gainLinear_=0.5, float Kp_Angular_ = 0.3, float toleranceLinear_ = 0.01, float toleranceAngular_ = 0.005, float lin=0.45, float ang=1, float acc_lin=0.7, float acc_ang=2.5, float frequency=50.0) {
        Kp_Linear = 10;
        Ki_Linear = 0.0;
        Kd_Linear = 0.0;
        Kp_Angular = 10;
        Ki_Angular = 0.0;
        Kd_Angular = 0.0;
        /* P_Gain = 50.0; 
        I_Gain = 0.0;
        D_Gain = 1000000.0; // 나중에 파라미터로 설정 */
        toleranceLinear = 0.001;
        toleranceAngular = 0.001;
        received = false;
        linearMax = lin;
        AngularMax = ang;
        pi = 3.141592;
        moveCase = 0;
        acc_linear = acc_lin;
        acc_angular = acc_ang;
        velocity_command_frequency = frequency;
        waypoint_num = 1;

        tar = cv::Mat_<float>(3, 1);
        cur = cv::Mat_<float>(3, 1);
        cur_vel = cv::Mat_<float>(2, 1);
        command = cv::Mat_<float>(2, 1);
        waypoints = cv::Mat_<float>(2, 4); // 4는 시작지점을 포함한 웨이포인트 개수
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
    float dX(float targetX) const { 
        return targetX - cur.at<float>(0); 
    }
    float dY(float targetY) const { 
        return targetY - cur.at<float>(1);
    }
    float dTheta(float targetTheta) {
        if (targetTheta - cur.at<float>(2) > pi) {
            return (targetTheta - cur.at<float>(2)) - 2 * pi;
        }
        else if (targetTheta - cur.at<float>(2) <= - pi) {
            return (targetTheta - cur.at<float>(2)) + 2 * pi;
        }
        else { 
            return targetTheta - cur.at<float>(2); 
        }
    }

    float distance(float targetX, float targetY){ // 목적지에서 실제 경로에 수선의 발을 내렸을 때 현재 위치와 수선의 발까지의 거리
        return sqrt( pow(dX(targetX), 2) + pow(dY(targetY), 2) ) * cos( dTheta(targetTheta) );
    }
    
    float error(){ // 진행 경로에서 얼마나 벗어나는 지
        float x1 = waypoints.at<float>(0, waypoint_num-1);
        float y1 = waypoints.at<float>(1, waypoint_num-1);
        float x2 = waypoints.at<float>(0, waypoint_num);
        float y2 = waypoints.at<float>(1, waypoint_num);
        float curX = cur.at<float>(0);
        float curY = cur.at<float>(1);
        // std::cout << "x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2 << std::endl;
        // std::cout << "curX: " << curX << " curY: " << curY << std::endl; 
        
        float a = (y2 - y1);
        float b = - (x1 - x2);
        float c = - (y2 - y1) * x1 + y1 * (x2 - x1);
        // std::cout << "a: " << a << " b: " << b << " c: " << c << std::endl;

        cv::Mat vector1 = (cv::Mat_<float>(3, 1) << curX - x1, curY - y1, 0.0); // 출발 점 기준으로 현재 위치 벡터
        // std::cout << "vector1: " << vector1 << std::endl;
        cv::Mat vector2 = (cv::Mat_<float>(3, 1) << x2 - x1, y2 - y1, 0.0); // 출발 점 기준으로 도착 점 벡터
        // std::cout << "vector2: " << vector2 << std::endl;
        cv::Mat result = vector1.cross(vector2); // 두 벡터를 외적해서 z의 값이 양수인지 음수인지 확인
        // std::cout << "result: " << result << std::endl;

        if ( sqrt(pow(a, 2) + pow(b, 2)) == 0 ) { // 아래 식에서 분모가 0이 되는 경우 방지
            return 0;
        }
        if ( result.at<float>(2) >= 0 ) { // 양수이면 진행 경로의 오른쪽에 있음
            return abs(a * curX + b * curY + c) / sqrt(pow(a, 2) + pow(b, 2));
        }
        else { // 음수이면 진행 경로의 왼쪽에 있음
            return - abs(a * curX + b * curY + c) / sqrt(pow(a, 2) + pow(b, 2));
        }
    }

    void moveLinear(float targetX, float targetY){
        float v = Kp_Linear * distance(targetX, targetY) + Ki_Linear * sum_distance + Kd_Linear * (distance(targetX, targetY) - past_distance) * velocity_command_frequency;
        
        if (v >= linearMax) {
            command.at<float>(0) = linearMax;
        }
        else if (v <= - linearMax) {
            command.at<float>(0) = - linearMax;
        }
        else {
            command.at<float>(0) = v;
        }

        if ( dX(waypoints.at<float>(1, waypoint_num)) == 0){ // atan2 계산 시 분모가 0이 되는 경우 방지
            if ( dY(waypoints.at<float>(0, waypoint_num)) >= 0) {
                targetTheta = pi / 2;
            }
            else {
                targetTheta = - pi / 2;
            }
        }
        else {
            targetTheta = atan2(dY(waypoints.at<float>(1, waypoint_num)), dX(waypoints.at<float>(0, waypoint_num)));
        }
        moveAngular(targetTheta);

        sum_distance += distance(targetX, targetY);
        past_distance = distance(targetX, targetY);
        
        // u(t) = Kp * e(t) + Kd * (e(t) - e(t-1)) / dt
        /* float cur_error = error();
        command.at<float>(1) = P_Gain * cur_error + (cur_error - past_error) * velocity_command_frequency;
        past_error = cur_error; */
    }
   
    void moveAngular(float targetTheta) {
        float w = Kp_Angular * dTheta(targetTheta) + Ki_Angular * sum_dTheta + Kd_Angular * (dTheta(targetTheta) - past_dTheta) * velocity_command_frequency;
        // 이 값이 AngularMax와 -AngularMax 사이에 오도록 함
        if (w >= AngularMax) {
            command.at<float>(1) = AngularMax;
        }
        else if (w <= -AngularMax) {
            command.at<float>(1) = - AngularMax;
        }
        else {
            command.at<float>(1) = w;}

        sum_dTheta += dTheta(targetTheta);
        past_dTheta = dTheta(targetTheta);
    }

    void moveToTarget() {
        switch (moveCase) {
            case 1:{ // waypoints 생성
                setWaypoint();
                std::cout << waypoints << std::endl;
                moveCase = 2;
                break;
            }
            
            case 2:{ // I, D 제어를 위한 초기화
                past_dTheta = 0;
                sum_dTheta = 0;
                moveCase = 3;
                break;
            }
            
            case 3:{ // waypoint를 바라보도록 회전      
                if ( dX(waypoints.at<float>(1, waypoint_num)) == 0 ) { // atan2 계산 시 분모가 0이 되는 경우 방지
                    if ( dY(waypoints.at<float>(0, waypoint_num)) >= 0 ) {
                        targetTheta = pi / 2;
                    }
                    else {
                        targetTheta = - pi / 2;
                    }
                }
                else {
                    targetTheta = atan2(dY(waypoints.at<float>(1, waypoint_num)), dX(waypoints.at<float>(0, waypoint_num)));
                }
                
                if (abs(dTheta(targetTheta)) >= toleranceAngular) {
                    moveAngular(targetTheta);
                }
                else {
                    moveCase = 4;
                }
                break;
            }

            case 4:{ // I, D 제어를 위한 초기화
                past_distance = 0;
                sum_distance = 0;
                moveCase = 5;
                break;
            }

            case 5:{ // waypoint로 직진
                float targetX = waypoints.at<float>(0, waypoint_num);
                float targetY = waypoints.at<float>(1, waypoint_num);
                if ( abs(distance(targetX, targetY)) > toleranceLinear ) {  
                    moveLinear(targetX, targetY);
                    // std::cout << "error: " << error() << std::endl;
                }
                else {
                    command.at<float>(0) = 0;
                    command.at<float>(1) = 0;
                    if ( waypoint_num < waypoints.cols - 1 ) {
                        std::cout << waypoints.cols - 1 << "개의 웨이포인트 중에 " << waypoint_num << "번째 웨이포인트 도착" << std::endl; 
                        waypoint_num++;
                        moveCase = 2;
                    }
                    else { // 최종 목적지 도달
                        std::cout << "********* 최종 목적지 도착 **********" << std::endl;
                        waypoint_num = 1;
                        moveCase = 0;
                    }
                } 
                break;
            }

        }
    }
};

#endif // AGV_CONTROLLER_HPP
