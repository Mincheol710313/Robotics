#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's move your robot")
    # python에서 input를 받을 경우에 문자열로 받기 때문에 형변환 필요
    speed = float(input("Input your speed: "))
    distance = float(input("Type your distance: "))
    isForward = int(input("Forward? "))

    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    vel_msg.linear.y = 0
    vel_msg.linear.y = 0        
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0        
    vel_msg.angular.z = 0       

    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance < int(distance)):
            velocity_publisher.publish(vel_msg)

            t1 = rospy.Time.now().to_sec()

            current_distance = speed * (t1-t0)

        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass
