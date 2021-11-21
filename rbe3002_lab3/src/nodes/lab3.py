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

    # test plz work

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
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.request_path)


        rospy.sleep(1)



    def request_path(self, msg):
        """
        Requests the path from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the path")
        rospy.wait_for_service('plan_path')
        try: 
            get_plan = rospy.ServiceProxy('plan_path', GetPlan)
            req = GetPlan()

            start_pose = Pose()
            start_pose.position.x = self.px
            start_pose.position.y = self.py
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            start_pose_stamped = PoseStamped(h, start_pose)

            goal_pose = msg.pose
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            goal_pose_stamped = PoseStamped(h, goal_pose)

            req.start = start_pose_stamped
            req.goal = goal_pose_stamped
            req.tolerance = 0
            resp = get_plan(req.start, req.goal, req.tolerance)
            rospy.loginfo("Got path succesfully")
            return resp
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


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab3().run()

