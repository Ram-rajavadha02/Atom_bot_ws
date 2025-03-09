#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose

def pose_changed(current_pose, last_pose, threshold=0.0001):
    # Check if the Euclidean distance between two poses is beyond the threshold
    return ((current_pose.position.x - last_pose.position.x)**2 +
            (current_pose.position.y - last_pose.position.y)**2 +
            (current_pose.position.z - last_pose.position.z)**2) > threshold**2

def get_atom_pose():
    rospy.init_node('get_atom_pose', anonymous=True)

    # Service client for getting model state
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # Model name for TurtleBot3
    model_name = 'atom'

    # Initialize last pose with a default value
    last_pose = Pose()

    rate = rospy.Rate(1)  # Set the loop rate (e.g., 1 Hz)

    while not rospy.is_shutdown():
        try:
            # Request to get the state of the TurtleBot3 model
            model_state = get_model_state(model_name, 'world')
            
            rospy.loginfo(f"atom Pose - X: {model_state.pose.position.x}, Y: {model_state.pose.position.y}, Z: {model_state.pose.position.z}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        rate.sleep()

if __name__ == '__main__':
    get_atom_pose()
