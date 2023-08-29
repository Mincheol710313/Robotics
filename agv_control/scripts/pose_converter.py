#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D, Twist

def turtlesim_pose_callback(data):
    # Callback function to receive turtlesim Pose message
    pose2d_msg = Pose2D()
    pose2d_msg.x = data.x
    pose2d_msg.y = data.y
    pose2d_msg.theta = data.theta

    twist_msg = Twist()
    twist_msg.linear.x = data.linear_velocity
    twist_msg.angular.z = data.angular_velocity

    pose2d_pub.publish(pose2d_msg)
    twist_pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('pose_converter', anonymous = True)
    
    pose2d_pub = rospy.Publisher('/pose', Pose2D, queue_size = 1)
    twist_pub = rospy.Publisher('/cur_vel', Twist, queue_size = 1)
    
    rospy.Subscriber('/turtle1/pose', Pose, turtlesim_pose_callback)
    
    rospy.spin()