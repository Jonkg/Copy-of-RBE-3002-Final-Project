#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from KBHit import KBHit
from coord import Coord
from rbe3002_lab4.srv import GetPose, NavToPose, BestFrontier, FollowPath
from tf.transformations import euler_from_quaternion
from state_machine import StateMachine
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_srvs.srv._Empty import Empty



class Lab4:

    def __init__(self):
        ## Initialize node
        rospy.init_node("lab4")

        ## Declare variables
        self.initial_pose = None
        self.mapdata = None
        self.goal_pose = None
        self.goal_set = False

        ## Subscribe to '/move_base_simple/goal' topic to get final psoe for Phase 3
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_goal)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1)

        ## Wait for services to startup
        rospy.wait_for_service('nav_to_pose')
        rospy.wait_for_service('best_frontier')
        rospy.wait_for_service('get_cspace')
        rospy.loginfo("Lab4 node ready")



    def set_goal(self, msg):
        pose = GetPose()
        pose.x = msg.pose.position.x
        pose.y = msg.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        pose.th = yaw
        self.goal_pose = pose
        self.goal_set = True



    def get_curr_pose(self):
        rospy.wait_for_service('get_pose')
        get_pose = rospy.ServiceProxy('get_pose', GetPose)
        try:
            resp = get_pose()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp



    def follow_path(self, path):
        follow_path = rospy.ServiceProxy('follow_path', FollowPath)
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        msg = Path(h, path.path.poses)
        try:
            resp = follow_path(msg)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp



    def nav_to_pose(self, x, y, th):
        nav_to_pose = rospy.ServiceProxy('nav_to_pose', NavToPose)
        try:
            resp = nav_to_pose(x, y, th)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp



    def get_path_to_frontier(self, x, y):
        best_frontier_path = rospy.ServiceProxy('best_frontier', BestFrontier)
        try:
            resp = best_frontier_path(x, y)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp



    def Idle(self):
        ## If keyboard press: save initial pose and start Phase 1

        kb = KBHit()
        if kb.kbhit():
            self.initial_pose = self.get_curr_pose()
            newState = "phase1"
        else:
            newState = "idle"
        kb.set_normal_term()

        return newState
        


    def PhaseOne(self):
        print("PhaseOne state!")    # Comment for troubleshooting purposes

        ## Explore unknown map and save map to file

        ## Get current pose from 'navigator' node
        curr_pose = self.get_curr_pose()
        ## Get centroid of best frontier from 'frontier explorer'
        path = self.get_path_to_frontier(curr_pose.x, curr_pose.y)
        ## If frontiers to explore: Command 'navigator' node to drive to frontier centroid
        if(path.exists):
            newState = "phase1"    
            self.follow_path(path)
            print("Navigate to pose")
        else:
            newState = "phase2"
            print("Finished generating map!")

        return newState



    def PhaseTwo(self):
        print("PhaseTwo state!")    # Comment for troubleshooting purposes

        print("Go to:")

        ## Navigate back to starting pose

        ## Make service call in if statement below to navigator node with desired pose as input
            ## Service would check if within tolerance and return bool
            ##      If within tolerance, stop and return true
            ##      Else, set wheel speeds for go to pose and return false

        resp = self.nav_to_pose(self.initial_pose.x, self.initial_pose.y, self.initial_pose.th)
        if(resp.reachedGoal):
            print("Arrived at destination!")
            newState = "phase3"
            ### SAVE THE MAP HERE ###
        else:
            newState = "phase2"
        
        return newState



    def PhaseThree(self):
        print("PhaseThree state!")  # Comment for troubleshooting purposes
        rospy.sleep(5)

        ## Navigate to selected goal pose

        ## Make service call in if statement below to navigator node with desired pose as input
            ## Service would check if within tolerance and return bool
            ##      If within tolerance, stop and return true
            ##      Else, set wheel speeds for go to pose and return false

        if(self.goal_set):
            if(self.nav_to_pose(self.goal_pose.x, self.goal_pose.y, self.goal_pose.th)):
                newState = "end"
                print("Arrived at destination!")
            else:
                newState = "phase3"
        else:
            newState = "phase3"

        return newState



    def run(self):
        sm = StateMachine()
        sm.add_state("idle", self.Idle)
        sm.add_state("phase1", self.PhaseOne)
        sm.add_state("phase2", self.PhaseTwo)
        sm.add_state("phase3", self.PhaseThree)
        sm.add_state("end", None, end_state = 1)
        sm.set_start("idle")
        print("Start!")
        sm.run()



if __name__ == '__main__':
    Lab4().run()
