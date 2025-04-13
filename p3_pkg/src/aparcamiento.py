#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import math
from tf.transformations import euler_from_quaternion

current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

def odom_callback(msg):
    global current_pose
    current_pose['x'] = msg.pose.pose.position.x
    current_pose['y'] = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    current_pose['yaw'] = yaw

def move_distance(distance, speed=0.2):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    start_x = current_pose['x']
    start_y = current_pose['y']
    twist = Twist()
    twist.linear.x = speed if distance > 0 else -speed
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        dx = current_pose['x'] - start_x
        dy = current_pose['y'] - start_y
        dist_travelled = math.sqrt(dx**2 + dy**2)
        if dist_travelled >= abs(distance):
            break
        pub.publish(twist)
        rate.sleep()
    twist.linear.x = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)

def turn_angle(angle, angular_speed=1.0):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    start_yaw = current_pose['yaw']
    twist = Twist()
    twist.angular.z = angular_speed if angle > 0 else -angular_speed
    rate = rospy.Rate(50)
    def angle_diff(a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2*math.pi
        while diff < -math.pi:
            diff += 2*math.pi
        return diff
    while not rospy.is_shutdown():
        diff = angle_diff(current_pose['yaw'], start_yaw)
        if abs(diff) >= abs(angle):
            break
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('movimiento', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.sleep(1)
    move_distance(1.5)
    turn_angle(math.radians(-90))
    move_distance(1.5)
