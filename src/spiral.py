#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

current_x = 0.0
current_y = 0.0
current_yaw = 0.0

def get_yaw(orientation):
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return yaw

def odom_callback(msg):
    global current_x, current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    current_yaw = get_yaw(msg.pose.pose.orientation)

def move_to_point(pub, target_x, target_y):
    global current_x, current_y, current_yaw

    # Calculate angle and distance to the target point
    target_angle = math.atan2(target_y - current_y, target_x - current_x)
    distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

    # Adjust the robot's orientation
    angular_speed = 0.3
    while abs(current_yaw - target_angle) > 0.01:
        twist_msg = Twist()
        twist_msg.angular.z = angular_speed if target_angle > abs(current_yaw) else -angular_speed
        pub.publish(twist_msg)
        rospy.sleep(0.1)

    # Stop rotating
    twist_msg = Twist()
    pub.publish(twist_msg)

    # Move forward to the target point
    linear_speed = 0.2
    twist_msg = Twist()
    twist_msg.linear.x = linear_speed
    while distance_to_target > 0.1:
        pub.publish(twist_msg)
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        rospy.sleep(0.1)

    # Stop moving
    twist_msg = Twist()
    pub.publish(twist_msg)

def zigzag_coverage(pub, initial_coordinates, final_coordinates):
    for i in range(len(final_coordinates)):
        move_to_point(pub, final_coordinates[i][0], final_coordinates[i][1])
        print(final_coordinates[i][0])
        print(final_coordinates[i][1])
        rospy.sleep(1)

        if i < len(initial_coordinates):
            move_to_point(pub, initial_coordinates[i][0], initial_coordinates[i][1])
            print(initial_coordinates[i][0])
            print(initial_coordinates[i][1])
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('zigzag_coverage', anonymous=True)
        pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/atom/odom', Odometry, odom_callback)

        # Define the coordinates to cover in a zigzag path
        initial_coordinates = [(0,0)]
        final_coordinates = [(5,5)]

        # Move the robot in a zigzag path
        zigzag_coverage(pub, initial_coordinates, final_coordinates)

    except rospy.ROSInterruptException:
        pass
