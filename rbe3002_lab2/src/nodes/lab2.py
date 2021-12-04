#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, projection_matrix

class Lab2:

    # test plz work

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT name it 'lab2'
        rospy.init_node("lab2")

        self.px = 0
        self.py = 0
        self.pth = 0

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)


        rospy.sleep(1)


    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # Create twist message
        msg_cmd_vel = Twist()
        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        # Send command
        self.cmd_vel.publish(msg_cmd_vel)

        return msg_cmd_vel

        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT

        initialX = self.px
        initialY = self.py

        targetXRoboFrame = distance
        currentXRoboFrame = 0

        tolerance = 0.05
        kp = 8

        while (abs(targetXRoboFrame - currentXRoboFrame) > tolerance):
            currentXRoboFrame = math.cos(self.pth)*(self.px-initialX) + math.sin(self.pth)*(self.py-initialY)
            currentYRoboFrame = -math.sin(self.pth)*(self.px-initialX) + math.cos(self.pth)*(self.py-initialY)
            self.send_speed(linear_speed, currentYRoboFrame*kp)
            rospy.sleep(0.05)

        self.send_speed(0, 0)


    def boundAngle(self, angle):
        if(angle > 0):
            while (angle > math.pi):
                angle = angle - 2*math.pi   
        else:   
            while (angle < -math.pi):
                angle = angle + 2*math.pi
        return angle



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        initialAngle = self.pth
        targetAngle = self.boundAngle(initialAngle + angle)

        if (angle < 0):
            aspeed = -aspeed

        tolerance = 0.05

        while (abs(targetAngle - self.pth) > tolerance):
             self.send_speed(0, aspeed)
             rospy.sleep(0.05)
        self.send_speed(0, 0)



    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT

        # rotate to look at the target location
        targetX = msg.pose.position.x
        targetY = msg.pose.position.y

        deltaX = targetX - self.px
        deltaY = targetY - self.py

        lookAngle = math.atan2(deltaY, deltaX)
        lookAngleError = self.boundAngle(lookAngle - self.pth)
        self.rotate(lookAngleError, 0.5)

        # travel to the target distance
        targetDistance = math.sqrt(deltaX**2 + deltaY**2)
        tolerance = 0.1
        kp = 5

        while (targetDistance > tolerance):
            self.send_speed(0.25, lookAngleError*kp)
            targetDistance = math.sqrt((targetX - self.px)**2 + (targetY - self.py)**2)
            lookAngle = math.atan2(deltaY, deltaX)
            lookAngleError = lookAngle - self.pth
            rospy.sleep(0.05)

        #rotate to the final angle needed for the final pose
        poseAngle = msg.pose.orientation
        quaternion_angles = [poseAngle.x, poseAngle.y, poseAngle.z, poseAngle.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_angles)

        targetAngle = self.boundAngle(yaw - self.pth)

        self.rotate(targetAngle, 0.5)




    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw




    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    #Lab2().drive(1.0, 0.5)
    #Lab2().rotate((math.pi/2), 1.0)
    #rospy.sleep(2)
    #Lab2().rotate(-(math.pi/2), 1.0)
    Lab2().run()

