import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# Global variables
obstacle_detected = False
closest_obstacle_angle = 0.0
closest_obstacle_distance = 0.0
obstacle_distances = []
pid_parameters = {'kp': 1.0, 'ki': 0.0, 'kd': 0.1}  # Adjust these parameters as needed

# PID controller variables
previous_error = 0.0
integral = 0.0

def scan_callback(msg):
    global obstacle_detected, closest_obstacle_angle, closest_obstacle_distance, obstacle_distances
    obstacle_distances = msg.ranges

    # Check if any distance measurement is less than the threshold based on specified ranges
    if -1.57 <= closest_obstacle_angle <= -1.0 or 1.0 <= closest_obstacle_angle <= 1.57:
        obstacle_detected = any(0.0 < dist < 0.5 for dist in obstacle_distances)
    else:
        obstacle_detected = any(0.0 < dist < 0.7 for dist in obstacle_distances)

    if obstacle_detected:
        closest_obstacle_index = obstacle_distances.index(min(obstacle_distances))
        closest_obstacle_angle = msg.angle_min + closest_obstacle_index * msg.angle_increment
        closest_obstacle_distance = obstacle_distances[closest_obstacle_index]
    else:
        closest_obstacle_angle = 0.0
        closest_obstacle_distance = 0.0


def calculate_pid(twist, target_angle):
    global previous_error, integral, pid_parameters

    # PID parameters
    kp = pid_parameters['kp']
    ki = pid_parameters['ki']
    kd = pid_parameters['kd']

    # Calculate error
    error = target_angle - closest_obstacle_angle

    # Proportional term
    proportional = kp * error

    # Integral term
    integral = integral + ki * error

    # Derivative term
    derivative = kd * (error - previous_error)

    # PID control
    angular_velocity = proportional + integral + derivative

    # Update previous error for the next iteration
    previous_error = error

    return angular_velocity

def avoid_obstacles():
    rospy.init_node('obstacle_avoidance', anonymous=True)
    pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.loginfo("Obstacle Avoidance for TurtleBot3")
    rospy.loginfo("Press 'Ctrl + C' to exit.")

    twist = Twist()

    try:
        while not rospy.is_shutdown():
            if obstacle_detected:
                left_obstacle = any(dist < 1.0 for dist in obstacle_distances[:len(obstacle_distances)//2])
                right_obstacle = any(dist < 1.0 for dist in obstacle_distances[len(obstacle_distances)//2:])

                if left_obstacle and right_obstacle:
                    twist.linear.x = -0.3  # Reverse
                    target_angle = math.pi  # Target angle for turning while reversing
                else:
                    twist.linear.x = 0.2  # Move forward
                    target_angle = 0.5 if closest_obstacle_angle < 0 else -0.5  # Target angle for turning away from the obstacle

                twist.angular.z = calculate_pid(twist, target_angle)
            else:
                twist.linear.x = 0.2  # Move forward
                twist.angular.z = 0.0

            pub.publish(twist)
            rospy.sleep(0.1)

    except KeyboardInterrupt:
        rospy.loginfo("Shutdown requested. Stopping obstacle avoidance.")
    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.loginfo("Obstacle avoidance stopped.")

if __name__ == '__main__':
    obstacle_detected = False
    closest_obstacle_angle = 0.0
    closest_obstacle_distance = 0.0
    obstacle_distances = []
    try:
        avoid_obstacles()
    except rospy.ROSInterruptException:
        pass

