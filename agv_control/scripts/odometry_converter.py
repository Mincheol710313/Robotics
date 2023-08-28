#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from math import atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist

def odom_callback(odom_msg):
    # Extract position and orientation from Odometry message
    pose = odom_msg.pose.pose
    position = pose.position
    orientation = pose.orientation

    # Extract linear and angular velocities from Twist message
    twist = odom_msg.twist.twist
    linear_velocity = twist.linear
    angular_velocity = twist.angular

    # Create Pose2D message
    estimated_pos = Pose2D()
    estimated_pos.x = position.x
    estimated_pos.y = position.y
    estimated_pos.theta = 2 * atan2(orientation.z, orientation.w)  # Calculate yaw angle
    # theta가 -pi에서 pi 사이의 값을 가지도록 조정
    if estimated_pos.theta > 3.141592:
        estimated_pos.theta -= 2 * 3.141592
    elif estimated_pos.theta < -3.141592:
        estimated_pos.theta += 2 * 3.141592
    else:
        pass

    # Create Twist message
    cur_vel = Twist()
    cur_vel.linear = linear_velocity
    cur_vel.angular = angular_velocity

    # Publish Pose2D and Twist messages
    estimated_pos_pub.publish(estimated_pos)
    cur_vel_pub.publish(cur_vel)

rospy.init_node('odometry_converter')

# Initialize publishers
estimated_pos_pub = rospy.Publisher('pos', Pose2D, queue_size=1)
cur_vel_pub = rospy.Publisher('cur_vel', Twist, queue_size=1)

# Subscribe to Odometry topic
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

rospy.spin()  # Keep the node running
