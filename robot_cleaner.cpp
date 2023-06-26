#include "ros/ros.h"
#include "geometry_msgs/Twist.h" //geometry_msgs의 Twist Type을 사용하기 위한 헤더파일
ros::Publisher velocity_publisher; 
// velocity에 대한 정보를 제공해주는 parameter를 설정하는 것, 이것을 turtlesim으로 설정해서 그것에 대한 값을 받아오려고 한다!

//method to move the robot straight
void move(double speed, double distance, bool isForward);

int main(int argc, char** argv){
    // 사용할 노드(프로그램)의 이름 설정, 파일 이름과 같게 설정
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n; // node를 다루는 인자 설정

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    // publisher를 선언, velocity_publisher로 geometry_msgs의 Twist 메세지 파일을 이용, 토픽명은 
    // "/turtle1/cmd_vel"이며, 이것과 같은 topic이 들어온 경우에 반환. 퍼블리ㅕ 큐 사이즈를 10개로 설정한다는 것
    

}

void move(double speed, double distance, bool isForward){
    geometry_msgs::Twist vel_msg;
    if (isForward)
        vel_msg.linear.x = abs(speed); // 우리가 새롭게 설정해줄 geometry_msgs::Twist의 vel_msg의 값을 설정
    else
        vel_msg.linear.x = -abs(speed);
    
    // 일직선으로 가기 위해서 y,z 방향 속도 0
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    // 회전을 없애기 위해서 x,y,z 방향의 회전 속도 0
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    double t0 = ros::Time::now().to_sec();
    current_distance = 0;

    while(current_distance < distance){
        // velocity 정보를 알려주는 publisher로 우리가 설정해주는 vel_msg를 publish 해준다.
        velocity_publisher.publish(vel_msg);

        t1 = ros::Time::Time.now().to_sec();
        current_distance = speed * (t1-t0);
    }
    
    // 움직인 후에
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}