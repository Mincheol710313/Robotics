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
    double velocity_command_frequency;
    double lookaheadDistance;
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
    CONTROL(
        double Kp_Linear_ = 1.7, double Ki_Linear_ = 0.0, double Kd_Linear_ = 0.0, 
        double Kp_Angular_ = 4.7, double Ki_Angular_ = 0.0, double Kd_angular_ = 0.0, 
        double toleranceLinear_ = 0.01, double toleranceAngular_ = 0.005, double linearMax_ = 0.45, double AngularMax_ = 1, 
        double velocity_command_frequency_ = 60.0, double lookaheadDistance_ = 1.0) {
        Kp_Linear = Kp_Linear_;
        Ki_Linear = Ki_Linear_;
        Kd_Linear = Kd_Linear_;
        Kp_Angular = Kp_Angular_;
        Ki_Angular = Ki_Angular_;
        Kd_Angular = Kd_angular_;
        toleranceLinear = toleranceLinear_;
        toleranceAngular = toleranceAngular_;
        linearMax = linearMax_;
        AngularMax = AngularMax_;
        velocity_command_frequency = velocity_command_frequency_;
        lookaheadDistance = lookaheadDistance_;
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

    double distance(double targetX, double targetY){ // 목적지까지의 거리
        vector<double> v1 = {avg_cur[0] - targetX, avg_cur[1] - targetY}; // 웨이포인트(목적지)에서 현재 위치를 바라보는 벡터
        vector<double> v2 = {waypoints[waypoint_num - 1][0] - targetX, waypoints[waypoint_num - 1][1] - targetY}; // 웨이포인트(목적지)에서 이전 웨이포인트(출발지)를 바라보는 벡터

        if (sqrt( pow(v2[0], 2) + pow(v2[1], 2) ) == 0) { // 분모가 0이 되는 경우 방지
            return 0;
        }
        else {
            return (v1[0] * v2[0] + v1[1] * v2[1]) / sqrt( pow(v2[0], 2) + pow(v2[1], 2) );
        }
    }

    vector<double> ahead_target(){ // 실시간으로 lookaheadDistance만큼의 거리에 있는 target을 구하는 함수
    // https://vinesmsuic.github.io/robotics-purepersuit/#Lookahead-Point 참고

        vector<double> d = 
        { waypoints[waypoint_num][0] - waypoints[waypoint_num - 1][0], waypoints[waypoint_num][1] - waypoints[waypoint_num - 1][1] };
        // 이전 웨이포인트(출발점)에서에서 현재 웨이포인트(도착점)를 바라보는 벡터
        vector<double> f = { waypoints[waypoint_num - 1][0] - avg_cur[0], waypoints[waypoint_num - 1][1] - avg_cur[1] };
        // 로봇의 현재 위치에서 이전 웨이포인트(출발점)를 바라보는 벡터
        
        double a = d[0] * d[0] + d[1] * d[1];
        double b = 2 * (f[0] * d[0] + f[1] * d[1]);
        double c = f[0] * f[0] + f[1] * f[1] - lookaheadDistance * lookaheadDistance;
        double discriminant = b * b - 4 * a * c;
        double t;

        double targetX;
        double targetY;

        if (a == 0) { // a로 나누는 경우 방지 (이전 웨이포인트와 현재 웨이포인트가 같을 때)
            targetX = waypoints[waypoint_num][0];
            targetY = waypoints[waypoint_num][1];
        }
        else if (discriminant < 0) {
            cout << "lookaheadDistance가 너무 작습니다." << endl;
            targetX = waypoints[waypoint_num][0];
            targetY = waypoints[waypoint_num][1];
        }
        else if ( sqrt(pow(dX(waypoints[waypoint_num][0]), 2) + pow(dY(waypoints[waypoint_num][1]), 2)) <= lookaheadDistance) { // 웨이포인트까지의 거리가 lookaheadDistance보다 가까운 경우
            targetX = waypoints[waypoint_num][0];
            targetY = waypoints[waypoint_num][1];
        }
        else {
            discriminant = sqrt(discriminant);
            double t1 = (- b - discriminant) / (2 * a);
            double t2 = (- b + discriminant) / (2 * a);

            if( (t2 >= 0) && (t2 <= 1) ) {
                targetX = waypoints[waypoint_num - 1][0] + d[0] * t2;
                targetY = waypoints[waypoint_num - 1][1] + d[1] * t2;
            }
            else {
                targetX = waypoints[waypoint_num][0];
                targetY = waypoints[waypoint_num][1];
            }
        }

        return {targetX, targetY};
    }

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

        if (distance(targetX, targetY) > 0.5) { // agv가 waypoint를 넘었을 때 갑자기 크게 회전하는 경우를 방지
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
                waypoint_num = 1;
                moveCase = 2;
                break;
            }
            
            case 2:{ // 웨이포인트를 생략할 지 결정
                if ( waypoints[waypoint_num] == waypoints[waypoint_num - 1] ) { // 이전 웨이포인트와 현재 웨이포인트가 같은 경우
                    moveCase = 7;
                }
                else if ( sqrt( pow(dX(waypoints[waypoint_num][0]), 2) + pow(dY(waypoints[waypoint_num][1]), 2) ) <= toleranceLinear ) { // 현재 위치가 현재 웨이포인트에 너무 가까운 경우
                    moveCase = 7;
                }
                else { moveCase = 3; }

                break;
            }
            case 3:{ // I, D 제어를 위한 초기화
                past_distance = 0;
                sum_distance = 0;
                past_dTheta = 0;
                sum_dTheta = 0;
                moveCase = 4;
                break;
            }

            case 4:{ // waypoint를 바라보도록 회전
                if (( abs(dTheta(targetTheta())) <= toleranceAngular ) && ( abs(cur_vel[1]) <= 0.01 )){
                    command[0] = 0;
                    command[1] = 0;
                    moveCase = 5;            
                }
                else {
                    moveAngular(targetTheta());
                }
                break;
            }
            
            case 5:{ // I, D 제어를 위한 초기화
                past_distance = 0;
                sum_distance = 0;
                past_dTheta = 0;
                sum_dTheta = 0;
                moveCase = 6;
                
                break;
            }

            case 6:{ // waypoint를 향해 직진
                if (( abs(distance(waypoints[waypoint_num][0], waypoints[waypoint_num][1])) <= toleranceLinear ) && ( abs(cur_vel[0]) <= 0.001 )) {  
                    command[0] = 0;
                    command[1] = 0;
                    moveCase = 7;
                }
                else {
                    double targetX = ahead_target()[0];
                    double targetY = ahead_target()[1];
                    moveLinear(targetX, targetY);
                } 
                break;
            }

            case 7:{ // 최종 목적지인지 아닌지 판별
                cout << "웨이포인트 도착 " << "(" << waypoint_num << "/" << waypoints.size() -1 << ")" << endl;
                if ( waypoint_num < waypoints.size() - 1 ) {
                    waypoint_num ++;
                    moveCase = 2;
                }
                else { // 최종 목적지 도달
                    cout << "최종 목적지 도착" << endl;
                    moveCase = 8;
                }
                break;
            }

            case 8:{ // 각도를 0으로 맞추기 위한 회전
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
        
        }
        // cout << "moveCase: " << moveCase << endl;
    }
};

#endif // AGV_CONTROLLER_HPP
