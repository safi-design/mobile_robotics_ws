#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

# PID Controller Parameters
kp = 1.0
ki = 0.0
kd = 0.1

# Wall-Following Parameters
obstacle_threshold = 0.55
move_speed = 0.2
rotation_speed = 0.45

# Initialize previous time
prev_time = None

# Initialize PID Controller Variables
prev_error = 0.0
integral = 0.0

# Callback function for Laser Scan data
def laser_callback(scan_msg):
    global prev_error, integral, prev_time

    cmd = Twist()
    ranges = np.array(scan_msg.ranges)
    ranges -= 0.180

    current_time = rospy.Time.now()
    if prev_time is not None:
        dt = (current_time - prev_time).to_sec()
    else:
        dt = 0.0
    prev_time = current_time

    if all(ranges[0:20] > obstacle_threshold) and all(ranges[-20:] > obstacle_threshold):
        cmd.linear.x = move_speed
        cmd.angular.z = 0.0
    else:
        front_distance = min(min(ranges[0:20]), min(ranges[-20:]))
        error = obstacle_threshold - front_distance

        integral += error * dt
        derivative = (error - prev_error) / dt if dt != 0 else 0.0
        output = kp * error + ki * integral + kd * derivative

        cmd.linear.x = move_speed / 2.0
        cmd.angular.z = output

    cmd_pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('wall_follower')

    # Initialize Publisher for Velocity Commands
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Subscribe to Laser Scan Data
    laser_sub = rospy.Subscriber('scan', LaserScan, laser_callback)

    # ROS Loop
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()
