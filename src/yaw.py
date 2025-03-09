#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def get_yaw(orientation):
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return yaw

def odom_callback(msg):
    orientation = msg.pose.pose.orientation
    current_yaw = get_yaw(orientation)
    print("Current Yaw Angle:", current_yaw)

if __name__ == '__main__':
    try:
        rospy.init_node('yaw_angle_reader', anonymous=True)
        rospy.Subscriber('/atom/odom', Odometry, odom_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

