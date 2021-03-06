#!/usr/bin/env python

import rospy
import math
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, projection_matrix
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped

class Lab3:

    def __init__(self):
        """
        Class constructor
        """
        #name of the node
        rospy.init_node("lab3")

        self.px = 0
        self.py = 0
        self.pth = 0

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.request_path
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_to_point)

        rospy.sleep(1)



    def request_path(self, msg):
        """
        Requests the path from path_planner
        :return [Path] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        ## Call 'plan_path" servie for path_planner node
        rospy.loginfo("Requesting the path")
        rospy.wait_for_service('plan_path')

        try: 
            ## Service handler
            get_plan = rospy.ServiceProxy('plan_path', GetPlan)          
            req = GetPlan()

            ## Construct PoseStamped msg for start pose
            start_pose = Pose()
            start_pose.position.x = self.px
            start_pose.position.y = self.py
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            start_pose_stamped = PoseStamped(h, start_pose)

            ## Construct PoseStamped msg for goal pose
            goal_pose = msg.pose
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            goal_pose_stamped = PoseStamped(h, goal_pose)

            ## Construct GetPlan msg for request
            req.start = start_pose_stamped
            req.goal = goal_pose_stamped
            req.tolerance = 0

            ## Return response from 'plan_path' service call
            resp = get_plan(req.start, req.goal, req.tolerance)
            rospy.loginfo("Got path succesfully")
            return resp.plan.poses

        except rospy.ServiceException as e:
             rospy.loginfo("Service call failed: %s"%e)


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

        ## Get initial position
        initialX = self.px
        initialY = self.py

        ## Set goal position
        targetXRoboFrame = distance
        currentXRoboFrame = 0

        ## Set constants
        tolerance = 0.05
        kp = 8

        ## Drive straight until within tolerance of target distance
        while (abs(targetXRoboFrame - currentXRoboFrame) > tolerance):
            currentXRoboFrame = math.cos(self.pth)*(self.px-initialX) + math.sin(self.pth)*(self.py-initialY)
            currentYRoboFrame = -math.sin(self.pth)*(self.px-initialX) + math.cos(self.pth)*(self.py-initialY)
            self.send_speed(linear_speed, currentYRoboFrame*kp)
            rospy.sleep(0.05)

        ## Stop
        self.send_speed(0, 0)


    ## Adjust angle to equivalent between -pi nad pi
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

        ## Change turn direction for negative angles
        if (angle < 0):
            aspeed = -aspeed

        tolerance = 0.05

        ## Turn towards target angle until within tolerance
        while (abs(targetAngle - self.pth) > tolerance):
            if (abs(targetAngle - self.pth) > 0.5):
                self.send_speed(0, aspeed)
            else:
                self.send_speed(0, aspeed/2)
            rospy.sleep(0.05)

        ## Stop
        self.send_speed(0, 0)


    ## Navigate to a goal 
    def nav_to_point(self, msg):
        """
        Navigates the robot to the specified point
        :param msg [Path] from 'plan_path' service
        """
        path = self.request_path(msg)
        ## Drive to each wapoint along path in sequence
        for pose in path:
            self.go_to(pose.pose.position.x, pose.pose.position.y)
        ## Turn to face final heading
        self.final_heading(pose.pose)


    ## Turn to face the heading given in a Pose message
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
        ### REQUIRED CREDIT

        deltaX = targetX - self.px
        deltaY = targetY - self.py

        ## Turn to face target position
        lookAngle = math.atan2(deltaY, deltaX)
        lookAngleError = self.boundAngle(lookAngle - self.pth)
        self.rotate(lookAngleError, 0.5)

        ## travel distance to the target position
        initialTargetDistance = math.sqrt(deltaX**2 + deltaY**2)
        targetDistance = initialTargetDistance
        tolerance = 0.05
        kp = 10

        ## Keep driving nad adjsuting heading until within toelrance of target position
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
        ## Update robot position and heading from odometry messages
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab3().run()

