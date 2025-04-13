#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MazeEscape:
    def __init__(self):
        rospy.init_node('maze_escape', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.twist = Twist()
        self.scan_data = None
        self.current_yaw = 0.0
        self.rate = rospy.Rate(10)
        self.state = 0
        self.wall_in_the_left = False
        self.forward_speed = 0.2
        self.turn_speed = 0.5
        self.front_threshold = 0.8
        self.left_threshold = 1.0
        self.desired_wall_distance = None 
        self.last_sensor_time = rospy.Time.now().to_sec()

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def scan_callback(self, msg):
        self.scan_data = msg.ranges

    def get_sector(self, start, end):
        if self.scan_data is None:
            return float('inf')
        sector = list(self.scan_data[start:end])
        valid = [d for d in sector if d > 0.01]
        return min(valid) if valid else float('inf')
    
    def turn_angle(self, angle, angular_speed=1.0):
        rospy.sleep(0.5)
        start_yaw = self.current_yaw
        twist = Twist()
        twist.angular.z = angular_speed if angle > 0 else -angular_speed
        rate = rospy.Rate(50)

        def angle_diff(a, b):
            diff = a - b
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        while not rospy.is_shutdown():
            diff = angle_diff(self.current_yaw, start_yaw)
            if abs(diff) >= abs(angle):
                break
            self.pub.publish(twist)
            rate.sleep()

        twist.angular.z = 0.0
        self.pub.publish(twist)
        rospy.sleep(0.5)

    def get_sector_left_from_front(self):
        self.turn_angle(math.radians(90), self.turn_speed)
        rospy.sleep(0.1)
        distance = min(self.get_sector(355, 360), self.get_sector(0, 5))
        self.turn_angle(math.radians(-90), self.turn_speed)
        rospy.sleep(0.1)
        rospy.loginfo("Left sensor measured distance: %.2f", distance)
        return distance

    def get_sector_both(self):
        front = min(self.get_sector(355, 360), self.get_sector(0, 5))
        left = self.get_sector_left_from_front()
        front_close = front < self.front_threshold
        left_close = left <= self.left_threshold
        if (left == float('inf')):
            left_close = True
        rospy.loginfo("Debug: front=%.2f, left=%.2f, front_close=%s, left_close=%s", front, left, str(front_close), str(left_close))
        return left_close, front_close

    def run(self):
        while not rospy.is_shutdown():
            if self.scan_data is None:
                self.rate.sleep()
                continue

            current_time = rospy.Time.now().to_sec()

            if self.state == 0:
                self.twist.linear.x = self.forward_speed
                self.twist.angular.z = 0.0
                front = min(self.get_sector(355, 360), self.get_sector(0, 5))
                rospy.loginfo("Step 1: front distance: %.2f", front)
                if front < self.front_threshold:
                    self.desired_wall_distance = front
                    self.state = 1
                    rospy.loginfo("Wall detected. Preparing initial right turn.")
            elif self.state == 1:
                self.turn_angle(math.radians(-90), self.turn_speed)
                self.wall_in_the_left = True
                self.state = 2
                self.last_sensor_time = current_time -8
                #rospy.loginfo("Step 1: last_sensor_time: %.2f", self.last_sensor_time)
                #rospy.loginfo("Initial right turn complete. Starting sensor checks.")
            elif self.state == 2:
                front_distance = min(self.get_sector(357, 360), self.get_sector(0, 3))
                #rospy.loginfo("Step 2: last_sensor_time: %.2f", self.last_sensor_time)
                #rospy.loginfo("Step 2: time: %.2f", current_time - self.last_sensor_time)
                if (current_time - self.last_sensor_time >= 12.0) or (front_distance < self.front_threshold):
                    left_close, front_close = self.get_sector_both()
                    self.last_sensor_time = current_time
                    rospy.loginfo("Sensor Check -> left_close: %s, front_close: %s", str(left_close), str(front_close))
                    if left_close and not front_close:
                        rospy.loginfo("Decision: move forward")
                        self.twist.linear.x = self.forward_speed
                        self.twist.angular.z = 0.0
                    elif left_close and front_close:
                        rospy.loginfo("Decision: turn right")
                        self.state = 3
                        self.turn_target = 'right'
                    elif (not left_close and not front_close) or (not left_close and front_close):
                        rospy.loginfo("Decision: turn left")
                        self.state = 3
                        self.turn_target = 'left'
                    else:
                        rospy.loginfo("Decision: default move forward")
                        self.twist.linear.x = self.forward_speed
                        self.twist.angular.z = 0.0
            elif self.state == 3:
                if self.turn_target == 'right':
                    self.turn_angle(math.radians(-90), self.turn_speed)
                elif self.turn_target == 'left':
                    self.turn_angle(math.radians(90), self.turn_speed)
                self.state = 2
                self.last_sensor_time = rospy.Time.now().to_sec() - 8
                rospy.loginfo("Turn complete, resuming sensor checks.")

            self.pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MazeEscape().run()
    except rospy.ROSInterruptException:
        pass
