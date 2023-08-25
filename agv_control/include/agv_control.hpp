#ifndef AGV_CONTROLLER_HPP
#define AGV_CONTROLLER_HPP
#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

class CONTROL {
    
private:
    // int --;
    bool received = false;
    double gainLinear;
    double Kp_Linear;
    double Ki_Linear;
    double Kd_Linear;
    double Kp_Angular;
    double Ki_Angular;
    double Kd_Angular;
    double toleranceLinear;
    double toleranceAngular;
    double linearMax;
    double AngularMax;
    double pi = 3.141592;
    int moveCase = 0;
    double acc_linear;
    double acc_angular;
    double velocity_command_frequency;
    double waypoint_num = 1; // 몇 번째 waypoint 인지
    double past_distance; // 이전 거리, D 제어를 하기 위함
    double sum_distance; // 거리의 합, I 제어를 하기 위함
    double past_dTheta; // 이전 각도, D 제어를 하기 위함
    double sum_dTheta; // 각도의 합, I 제어를 하기 위함

    vector<double> tar = {0.0, 0.0, 0.0};
    vector<double> cur = {0.0, 0.0, 0.0};
    vector<double> avg_cur = {0.0, 0.0, 0.0};
    vector<double> cur_vel = {0.0, 0.0, 0.0};
    vector<double> command = {0.0, 0.0};
    vector<vector<double>> waypoints = {
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}
        };
    vector<vector<double>> pose_5 = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };
    
public:
    CONTROL(double gainLinear_=0.5, double Kp_Angular_ = 0.3, double toleranceLinear_ = 0.01, double toleranceAngular_ = 0.005, double lin=0.45, double ang=1, double acc_lin=0.7, double acc_ang=2.5, double frequency=50.0) {
        Kp_Linear = 1.7;
        Ki_Linear = 0.0;
        Kd_Linear = 0.0;
        Kp_Angular = 4.7;
        Ki_Angular = 0.0;
        Kd_Angular = 0.0;
        toleranceLinear = 0.005;
        toleranceAngular = 0.005; // 나중에 파라미터로 설정
        linearMax = lin;
        AngularMax = ang;
        acc_linear = acc_lin;
        acc_angular = acc_ang;
        velocity_command_frequency = 60;
    };
    
    void setCur(vector<double> data) { cur = data; }
    void setCur_avg(){
        pose_5.erase(pose_5.begin()); // 가장 오래된 pose를 삭제
        pose_5.push_back({cur[0], cur[1], cur[2]}); // 현재 pose를 저장
        double avg_x = (pose_5[0][0] + pose_5[1][0] + pose_5[2][0] + pose_5[3][0] + pose_5[4][0]) / 5;
        double avg_y = (pose_5[0][1] + pose_5[1][1] + pose_5[2][1] + pose_5[3][1] + pose_5[4][1]) / 5;
        double avg_theta = (pose_5[0][2] + pose_5[1][2] + pose_5[2][2] + pose_5[3][2] + pose_5[4][2]) / 5;
        avg_cur = {avg_x, avg_y, avg_theta}; // 5개의 평균값을 이용하여 현재 위치를 설정
    }
    void setCur_Vel(vector<double> data) { cur_vel = data; } 
    void setTar(vector<double> data) { tar = data; } 
    void setReceived(bool data) { received = data; } 
    void startMove() { moveCase = 1; } 
    void setWaypoint() {
        waypoints = {
            {cur[0], cur[1]}, // 시작 지점 추가 (x: 시작 위치의 x값, y: 시작 위치의 y값)
            {cur[0], 0}, // 첫번째 웨이포인트 추가 (x: 시작 위치의 x값, y: 0)
            {tar[0], 0}, // 두번째 웨이포인트 추가 (x: 목표 위치의 x값, y: 0)
            {tar[0], tar[1]} // 세번째 웨이포인트 추가 (x: 목표 위치 x값, y: 목표 위치 y값)
            };
        
        cout <<"********** 출발 **********" << endl; // 웨이포인트 출력
        cout << "출발 지점: " << "(" << waypoints[0][0] << ", " << waypoints[0][1] << ")" << endl;
        cout << "첫번째 웨이포인트: " << "(" << waypoints[1][0] << ", " << waypoints[1][1] << ")" << endl;
        cout << "두번째 웨이포인트: " << "(" << waypoints[2][0] << ", " << waypoints[2][1] << ")" << endl;
        cout << "세번째 웨이포인트: " << "(" << waypoints[3][0] << ", " << waypoints[3][1] << ")" << endl;
        }

    vector<double> getCommand() const { return command; }
    double dX(double targetX) const { return targetX - avg_cur[0]; }
    double dY(double targetY) const { return targetY - avg_cur[1];}
    double dTheta(double targetTheta) { // 회전해야 할 각도
        if (targetTheta - avg_cur[2] > pi) {
            return (targetTheta - avg_cur[2]) - 2 * pi;
        }
        else if (targetTheta - avg_cur[2] <= - pi) {
            return (targetTheta - avg_cur[2]) + 2 * pi;
        }
        else { 
            return targetTheta - avg_cur[2]; 
        }
    }
    double targetTheta(){ // 목적지를 바라보는 각도
        if ( dX(waypoints[waypoint_num][1]) == 0){ // atan2 계산 시 분모가 0이 되는 경우 방지
            if ( dY(waypoints[waypoint_num][0]) >= 0) {
                return pi / 2;
            }
            else {
                return - pi / 2;
            }
        }
        else {
            return atan2(dY(waypoints[waypoint_num][1]), dX(waypoints[waypoint_num][0]));
        }
    }

    double distance(double targetX, double targetY){ // 목적지에서 현재 경로에 수선의 발을 내렸을 때 현재 위치와 수선의 발까지의 거리. 즉 이동해야 할 거리
        return sqrt( pow(dX(targetX), 2) + pow(dY(targetY), 2) ) * cos( dTheta(targetTheta()) );}

    void moveLinear(double targetX, double targetY){
        // u(t) = Kp * e(t) + Ki * sum(e(t)) * dt + Kd * (e(t) - e(t-1)) / dt
        double v = Kp_Linear * distance(targetX, targetY) + Ki_Linear * sum_distance / velocity_command_frequency + Kd_Linear * (distance(targetX, targetY) - past_distance) * velocity_command_frequency;
        
        if (v >= linearMax) {
            command[0] = linearMax;
        }
        else if (v <= - linearMax) {
            command[0] = - linearMax;
        }
        else {
            command[0] = v;
        }

        if (distance(targetX, targetY) > 0) { // agv가 waypoint를 넘었을 때 갑자기 뒤로 도는 경우를 방지
            moveAngular(targetTheta());
        }
        else {
            command[1] = 0;
        }

        sum_distance += distance(targetX, targetY);
        past_distance = distance(targetX, targetY);
        
    }
   
    void moveAngular(double targetTheta) {
        // u(t) = Kp * e(t) + Ki * sum(e(t)) * dt + Kd * (e(t) - e(t-1)) / dt
        double w = Kp_Angular * dTheta(targetTheta) + Ki_Angular * sum_dTheta / velocity_command_frequency + Kd_Angular * (dTheta(targetTheta) - past_dTheta) * velocity_command_frequency;
        
        // 각속도가 최대 각속도를 넘지 않도록 제한
        if (w >= AngularMax) {
            command[1] = AngularMax;
        }
        else if (w <= -AngularMax) {
            command[1] = - AngularMax;
        }
        else {
            command[1] = w;}

        sum_dTheta += dTheta(targetTheta);
        past_dTheta = dTheta(targetTheta);
    }

    void moveToTarget() {

        setCur_avg(); // 현재 위치를 5개의 평균값으로 설정

        switch (moveCase) {

            case 1:{ // waypoints 생성
                setWaypoint();
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
                if (( abs(dTheta(targetTheta())) <= toleranceAngular ) && ( abs(cur_vel[1]) <= 0.01 )){
                    command[0] = 0;
                    command[1] = 0;
                    moveCase = 4;            
                }
                else {
                    moveAngular(targetTheta()); 
                }
                break;
            }

            case 4:{ // I, D 제어를 위한 초기화
                past_distance = 0;
                sum_distance = 0;
                moveCase = 5;
                
                break;
            }

            case 5:{ // waypoint를 향해 직진
                if (( abs(distance(waypoints[waypoint_num][0], waypoints[waypoint_num][1])) <= toleranceLinear ) && ( abs(cur_vel[0]) <= 0.001 )) {  
                    cout << "웨이포인트 도착 " << "(" << waypoint_num << "/" << waypoints.size() -1 << ")" << endl;
                    command[0] = 0;
                    command[1] = 0;
                    moveCase = 6;
                }
                else {
                    moveLinear(waypoints[waypoint_num][0], waypoints[waypoint_num][1]);
                } 
                break;
            }

            case 6:{ // 최종 목적지인지 아닌지 판별
                if ( waypoint_num < waypoints.size() - 1 ) {
                    waypoint_num++;
                    moveCase = 2; // 원래 2, PID 튜닝할 때는 0으로 바꿔서 튜닝
                }
                else { // 최종 목적지 도달
                    cout << "********* 최종 목적지 도착 **********" << endl;
                    waypoint_num = 1;
                    moveCase = 7;
                }
                break;
            }

            case 7:{ // 각도를 0으로 맞추기 위한 회전
                if (( abs(dTheta(0)) <= toleranceAngular ) && ( abs(cur_vel[1]) <= 0.01 )){
                    command[0] = 0;
                    command[1] = 0;
                    cout << "********* 정지 **********" << endl;
                    moveCase = 0;            
                }
                else {
                    moveAngular(0); 
                }
                break;

            }
        // cout << "moveCase: " << moveCase << endl;
        }
    }
};

#endif // AGV_CONTROLLER_HPP
