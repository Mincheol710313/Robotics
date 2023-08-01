#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pi


class Robot:

    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cur_pose_subscriber = rospy.Subscriber('/estimated_pos', Pose, self.update_cur_pose_callback)     
        self.goal_pose_subscriber = rospy.Subscriber('/target_pos', Pose, self.update_goal_callback)

        self.goal_pose = Pose()
        self.cur_pose = Pose()
        self.rate = rospy.Rate(10)
        self.goal_received = False

    # pose 값을 구독할 때마다 업데이트
    def update_cur_pose_callback(self, data):
        self.cur_pose = data

    def update_goal_callback(self, data):
        self.goal_pose = data
        self.goal_received = True

    # 목표 x좌표와 현재 x좌표의 차이
    def d_x(self, goal_pose):
        return (goal_pose.x - self.cur_pose.x)

    # 목표 y좌표와 현재 y좌표의 차이    
    def d_y(self, goal_pose):
        return (goal_pose.y - self.cur_pose.y)
    
    # 목표 각도와 현재 각도의 차이
    # -pi <= d_theta <= pi
    def d_theta(self, goal_pose):
        if goal_pose.theta-self.cur_pose.theta >= pi:
            return (goal_pose.theta - self.cur_pose.theta) - 2 * pi
        elif goal_pose.theta-self.cur_pose.theta <= - pi:
            return (goal_pose.theta - self.cur_pose.theta) + 2 * pi
        else:
            return (goal_pose.theta - self.cur_pose.theta)

    # x축 linear 속도 (P 제어)
    def linear_vel_x(self, goal_pose, gainP = 1):
        return min(0.45,  gainP * abs(self.d_x(goal_pose))) # 최대 속도 설정 ex. 0.45m/s
    
    # y축 linear 속도 (P 제어)
    def linear_vel_y(self, goal_pose, gainP = 1):
        return min(0.45, gainP * abs(self.d_y(goal_pose))) # 최대 속도 설정 ex. 0.45m/s

    # theta angular 속도 (P 제어)
    def angular_vel(self, goal_pose, gainP = 1):
        return min(1, gainP * abs(self.d_theta(goal_pose)))

    def move2goal_x(self):
        distance_tolerance = 0.01

        vel_msg = Twist()

        while abs(self.d_x(self.goal_pose)) >= distance_tolerance:

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel_x(self.goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def move2goal_y(self):      
        distance_tolerance = 0.01

        vel_msg = Twist()

        while abs(self.d_y(self.goal_pose)) >= distance_tolerance:

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel_y(self.goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def move2goal_ang(self, ang):
        # 원하는 각도(global)로 회전
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.theta = ang * pi / 180

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.005

        vel_msg = Twist()

        while abs(self.d_theta(goal_pose)) >= distance_tolerance:

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            # d_theta 가 0보다 크면 시계반대방향으로 0보다 작으면 시계방향으로 회전
            if self.d_theta(goal_pose) >= 0:
                vel_msg.angular.z = self.angular_vel(goal_pose)
            else:
                vel_msg.angular.z = - self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def main(self):
        while not rospy.is_shutdown():
            if self.goal_received:
                if self.d_x(self.goal_pose)>=0:
                    self.move2goal_ang(0)
                else:
                    self.move2goal_ang(180)
                self.move2goal_x()
                if self.d_y(self.goal_pose)>=0:
                    self.move2goal_ang(90)
                else:
                    self.move2goal_ang(-90)
                self.move2goal_y()
                self.goal_received = False
            else:
                rospy.sleep(0.1)
            

if __name__ == '__main__':
    try:
        robot = Robot()
        robot.main()
        
    except rospy.ROSInterruptException:
        pass
