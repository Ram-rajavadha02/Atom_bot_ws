#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class GoalToGoal:
    def __init__(self, initial_x, initial_y, goal_x, goal_y):
        print('init')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False
        self.target_x = 0
        self.target_y = 0
        self.final_position_reached = False

        self.pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/atom/odom', Odometry, self.odom_callback)

        self.initial_x = initial_x
        self.initial_y = initial_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.linear_speed = 0.5
        self.angular_speed = 0.4

        self.generated_points = [(initial_x, initial_y)]  # Add the initial point
        self.generate_all_points()  # Call the function to generate points
    
    def get_yaw(orientation):
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, self.current_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                         msg.pose.pose.orientation.y,
                                                         msg.pose.pose.orientation.z,
                                                         msg.pose.pose.orientation.w])

    def orient_towards_goal(self):
        print('otg')
        target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)

        while not rospy.is_shutdown():
            # Align the robot's orientation with the goal direction
            angle_diff = target_angle - abs(self.current_yaw)
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

    def move_to_goal(self, target_x, target_y):
        print('mtg')
        # Update the target position in case it changes
        self.target_x = target_x
        self.target_y = target_y
        self.orient_towards_goal()

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

    def generate_all_points(self):
        print('gap')
        x, y = self.initial_x, self.initial_y
        step_size = self.goal_x - self.initial_x
        t = abs(self.goal_y - self.initial_y)
        t = int(t)
        if t % 2 == 0:
            loop_iterate = t + 1
        else:
            loop_iterate = t
        for i in range(loop_iterate):

            if i % 2 == 0:
                next_point = (x + step_size, y)
            else:
                next_point = (x, y)
            print(next_point)
            self.generated_points.append(next_point)

            if x + step_size == self.goal_x and y == self.goal_y:
                print('Goal reached. Stopping point generation.')
                break

            if step_size < 0:
                y = y - 1
            else:
                y += 1
            if i % 2 == 0:
                next_point = (x + step_size, y)
            else:
                next_point = (x, y)
            print(next_point)
            self.generated_points.append(next_point)

            if x == self.goal_x and y == self.goal_y:
                print('Goal reached. Stopping point generation.')
                break

    def travel_zigzag(self):
        print('tz')

        for i in range(len(self.generated_points)):
            self.move_to_goal(self.generated_points[i][0], self.generated_points[i][1])
            rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('goal_to_goal', anonymous=True)

    initial_x = float(input("Enter initial x-coordinate: "))
    initial_y = float(input("Enter initial y-coordinate: "))
    goal_x = float(input("Enter goal x-coordinate: "))
    goal_y = float(input("Enter goal y-coordinate: "))

    goal_to_goal = GoalToGoal(initial_x, initial_y, goal_x, goal_y)

    # Access the generated points
    print("Generated Points:", goal_to_goal.generated_points)

    # Move forward to the goal
    goal_to_goal.travel_zigzag()
