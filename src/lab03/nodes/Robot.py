#!/usr/bin/env python2

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        print("Starting....")
        rospy.sleep(1)
        print("Welcome back pilot, my controls are yours")

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### Make a new Twist message
        vel_msg = Twist()
        ### Publish the message
        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_speed
        self.vel_pub.publish(vel_msg)

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        starting_x = self.currentx
        if distance > 0: # Account for forward or backwards movement (+ = forwards, - = backwards
            drive_speed = linear_speed
        else:
            drive_speed = -1 * abs(linear_speed)
        while (abs(self.currentx - starting_x) < distance): #Publish distance only when the robot hasn't driven the distance yet
            self.send_speed(drive_speed, 0.0)

        self.send_speed(0.0, 0.0)  # Once done moving, then send a stop
        #print("All done driving")

    def rotate(self, angle, angular_speed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        starting_angle = self.yaw
        goal_angle = angle + starting_angle # The desired orientation is the current orientation plus the distance to go

        if angle > 0: # Account for clockwise or ccw turns (+ speed = ccw, - speed = cw)
            turn_speed = angular_speed
        else:
            turn_speed = -1 * abs(angular_speed)

        while abs(goal_angle - self.yaw) > 0.1:  # Publish distance only when the robot hasn't rotated to the correct angle yet
            self.send_speed(0.0, turn_speed)
        self.send_speed(0.0,0.0)  # Once done moving, then send a stop
        #print("All done rotating")


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        start_x = self.currentx # Determine current position
        start_y = self.currenty

        goal_x = msg.pose.position.x # Determine desired position
        goal_y = msg.pose.position.y

        goal_quat = msg.pose.orientation  # Pulls out the xyzw quaternion of the goal
        goalq = [goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w]  # Takes the xyzw quaternion and splits it apart
        (goal_roll, goal_pitch, goal_yaw) = euler_from_quaternion(goalq) #converts quaternion into 3D euler form

        drive_x = goal_x - start_x # Compute change in x and y between start and goal
        drive_y = goal_y - start_y
        distance = math.sqrt(pow(drive_x,2) + pow(drive_y,2)) # Compute straight line disance from start to goal (trig)
        theta = math.atan2(drive_y,drive_x) # Compute angle from start to goal (trig). atan2 prevents sign ambiguity

        first_theta = theta - self.yaw
        self.rotate(first_theta,0.1)
        print("First rotation done")
        rospy.sleep(0.5)
        self.drive(distance,0.2)
        print("Drive done")
        rospy.sleep(0.5)
        corrected_theta = goal_yaw - self.yaw
        self.rotate(corrected_theta,0.1)
        print("Last rotation done")


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        Updates the position in x and y, as well as updates the orientation in x,y,z and w (as a quaternion)
        """
        self.currentx = msg.pose.pose.position.x
        self.currenty = msg.pose.pose.position.y

        self.quat = msg.pose.pose.orientation #Pulls out the xyzw quaternion
        self.q = [self.quat.x, self.quat.y, self.quat.z, self.quat.w] #Takes the xyzw quaternion and splits it apart
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.q) #converts quaternion into 3D euler form

    def run(self):
        rospy.spin()

if __name__ == '__main__':
        robot = Lab2()
        rospy.spin()
        #robot.drive(1, 0.2)
        #robot.rotate(1, 0.2)
