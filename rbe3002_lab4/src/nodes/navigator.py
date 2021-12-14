#!/usr/bin/env python

import math
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from rbe3002_lab4.srv import GetPose, GetPoseResponse, NavToPose, NavToPoseResponse
from nav_msgs.msg import GridCells, OccupancyGrid, Odometry, Path
from nav_msgs.srv import GetMap, GetPlan
from tf.transformations import euler_from_quaternion, projection_matrix



class Navigator:

    def __init__(self):
        """
        Class constructor
        """
        #name of the node
        rospy.init_node("navigator")

        self.px = 0
        self.py = 0
        self.pth = 0

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry , self.update_odometry)

        ## Service to get current pose of the robot
        get_pose_service = rospy.Service('get_pose', GetPose, self.get_pose)

        ## Service to navigate to a desired pose
        nav_to_pose_service = rospy.Service('nav_to_pose', NavToPose, self.nav_to_pose)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1)

        ## Wait for services to startup
        rospy.wait_for_service('plan_path')
        rospy.loginfo("Navigator node ready")
    


    def get_pose(self, msg):
        resp = GetPoseResponse()
        resp.x = self.px
        resp.y = self.py
        resp.th = self.pth
        return resp



    def nav_to_pose(self, path_msg):

        path = path_msg.path.poses

        resp = NavToPoseResponse()

        if (len(path) > 4):
            self.go_to(path[3].pose.position.x, path[3].pose.position.y)
            self.go_to(path[4].pose.position.x, path[4].pose.position.y)
            self.go_to(path[5].pose.position.x, path[5].pose.position.y)
        else:
            for pose in path:
                self.go_to(pose.pose.position.x, pose.pose.position.y)

        self.stop()

        return resp



    def stop(self):
        """
        Stop
        """
        # Create twist message
        msg_cmd_vel = Twist()
        # Linear velocity
        msg_cmd_vel.linear.x = 0.0
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = 0.0

        # Send command
        self.cmd_vel.publish(msg_cmd_vel)

        return msg_cmd_vel



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
            if (abs(targetAngle - self.pth) > 0.5):
                self.send_speed(0, aspeed)
            else:
                self.send_speed(0, aspeed/2)
            rospy.sleep(0.05)
        self.send_speed(0, 0)



    def nav_to_point(self, path):
        """
        Navigates the robot to the specified point
        :param msg [PoseStamped]
        """
        for pose in path:
            self.go_to(pose.pose.position.x, pose.pose.position.y)
        self.final_heading(pose.pose)



    def final_heading(self, pose):
        poseAngle = pose.orientation
        quaternion_angles = [poseAngle.x, poseAngle.y, poseAngle.z, poseAngle.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_angles)

        targetAngle = self.boundAngle(yaw - self.pth)

        self.rotate(targetAngle, 1.0)



    def go_to(self, targetX, targetY):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        deltaX = targetX - self.px
        deltaY = targetY - self.py

        lookAngle = math.atan2(deltaY, deltaX)
        lookAngleError = self.boundAngle(lookAngle - self.pth)
        self.rotate(lookAngleError, 1.0)

        # travel to the target distance
        initialTargetDistance = math.sqrt(deltaX**2 + deltaY**2)
        targetDistance = initialTargetDistance
        tolerance = 0.05
        kp = 10

        while (targetDistance > tolerance):
            if (targetDistance < 0.5 or (initialTargetDistance - targetDistance) < 0.5):
                self.send_speed(0.1, lookAngleError*kp)
            else:
                self.send_speed(0.3, lookAngleError*kp)
            targetDistance = math.sqrt((targetX - self.px)**2 + (targetY - self.py)**2)
            lookAngle = math.atan2(deltaY, deltaX)
            lookAngleError = lookAngle - self.pth
            rospy.sleep(0.01)



    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        self.quat = quat_orig
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Navigator().run()

