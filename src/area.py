#!/usr/bin/env python

from turtle import st
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class GoalToGoal:
    def __init__(self, initial_x, initial_y, goal_x, goal_y):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False
        self.target_x = 0
        self.target_y = 0

        self.pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/atom/odom', Odometry, self.odom_callback)
        
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.linear_speed = 0.4
        self.angular_speed = 0.5
        print(self.current_x)
        print('init')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, self.current_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                         msg.pose.pose.orientation.y,
                                                         msg.pose.pose.orientation.z,
                                                         msg.pose.pose.orientation.w])
        print(self.current_x)
        print('oc')

    def orient_towards_goal(self):
        target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)

        while not rospy.is_shutdown():
            # Align the robot's orientation with the goal direction
            angle_diff = target_angle - self.current_yaw
            if abs(angle_diff) > 0.01:
                twist_msg = Twist()
                twist_msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                self.pub.publish(twist_msg)
                rospy.sleep(0.1)
            else:
                break

        # Stop rotating
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.sleep(1.0)
        print('otg')

    def move_to_goal(self):
        while not rospy.is_shutdown():
            # No obstacle, move forward to the goal
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed
            self.pub.publish(twist_msg)
            rospy.sleep(0.1)

            # Check if the goal is reached
            distance_to_goal = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
            if distance_to_goal < 0.2:
                rospy.loginfo("Goal reached. Stopping goal-to-goal navigation.")
                self.goal_reached = True
                break

        # Stop moving
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.sleep(1.0)
        print('mtg')

    def generate_next_point(self, current_point):
        x, y = current_point
        length = (math.sqrt((goal_x - initial_x)**2 + (goal_y - initial_y)**2))/(math.sqrt(2))
        step_size = length  # Adjust this based on the distance between points along the primary axis
        print(step_size)

        for i in range(10):
            if i % 2 == 0:
                next_point = (x + step_size, y)
            else:
                next_point = (x , y)

            self.target_x, self.target_y = next_point
            self.orient_towards_goal()

            # Move to the next point
            self.move_to_goal()

            # Switch direction for the next iteration
            y += 1
            if i % 2 == 0:
                next_point = (x + step_size, y)
            else:
                next_point = (x , y)
            self.target_x, self.target_y = next_point
            self.orient_towards_goal()
            self.move_to_goal()

            print(next_point)

        print('gnp')

    def travel_zigzag(self):

        while not rospy.is_shutdown():
            current_point = (self.current_x, self.current_y)
            self.generate_next_point(current_point)

            # Move to the next point
            self.move_to_goal()
            print('tz')

if __name__ == '__main__':
    rospy.init_node('goal_to_goal', anonymous=True)

    initial_x = float(input("Enter initial x-coordinate: "))
    initial_y = float(input("Enter initial y-coordinate: "))
    goal_x = float(input("Enter goal x-coordinate: "))
    goal_y = float(input("Enter goal y-coordinate: "))

    goal_to_goal = GoalToGoal(initial_x, initial_y, goal_x, goal_y)

    # Move forward to the goal
    goal_to_goal.travel_zigzag()
