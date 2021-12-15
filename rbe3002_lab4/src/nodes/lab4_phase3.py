#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from KBHit import KBHit
from coord import Coord
from lab4_util import Lab4Util
from rbe3002_lab4.srv import GetPose, NavToPose, Localize
from tf.transformations import euler_from_quaternion
from state_machine import StateMachine
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_srvs.srv._Empty import Empty



class Lab4_Phase3:

    def __init__(self):
        ## Initialize node
        rospy.init_node("lab4_phase3")

        ## Declare variables
        self.goal_pose = None
        self.goal_set = False

        ## Subscribe to '/move_base_simple/goal' topic to get final psoe for Phase 3
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_goal)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1)

        ## Wait for services to startup
        rospy.wait_for_service('nav_to_pose')
        rospy.wait_for_service('get_cspace')
        rospy.wait_for_service('get_pose')
        rospy.wait_for_service('localize')
        rospy.wait_for_service('global_localization')
        rospy.loginfo("Lab4 Phase 3 node ready")



    def set_goal(self, msg):
        pose = GetPose()
        pose.x = msg.pose.position.x
        pose.y = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        pose.th = yaw
        self.goal_pose = pose
        self.goal_set = True



    def get_curr_pose(self):
        get_pose = rospy.ServiceProxy('get_pose', GetPose)
        try:
            resp = get_pose()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp
    


    def init_localization(self):
        init_localization = rospy.ServiceProxy('global_localization', GetPose)
        try:
            resp = init_localization()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp



    def localization_routine(self, numTurns, angle, speed):
        localize = rospy.ServiceProxy('localize', Localize)
        try:
            resp = localize(numTurns, angle, speed)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp



    def nav_to_pose(self, x, y, th):
        nav_to_pose = rospy.ServiceProxy('nav_to_pose', NavToPose)
        goalPosWC = Point(x, y, 0)
        mapdata = self.get_cspace_map()
        goalPosGC = Lab4Util.world_to_grid(mapdata, goalPosWC)
        try:
            resp = nav_to_pose(goalPosGC.x, goalPosGC.y, th)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp

    

    def get_cspace_map(self):
        try: 
            get_cspace = rospy.ServiceProxy('get_cspace', GetMap)
            resp = get_cspace()
            return resp.map
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)



    def Idle(self):
        ## If keyboard press: save initial pose and start Phase 1

        kb = KBHit()
        if kb.kbhit():
            self.initial_pose = self.get_curr_pose()
            # self.init_localization()
            newState = "localize"
        else:
            newState = "idle"
        kb.set_normal_term()

        return newState

    

    def Localize(self):
        print("Localize state!")    # Comment for troubleshooting purposes

        ## Spin to give AMCL a chance to localize the robot

        resp = self.localization_routine(6, 2, 0.5)
        if(resp.finished):
            newState = "phase3"
        else:
            newState = "localize"
        
        return newState



    def PhaseThree(self):
        print("PhaseThree state!")    # Comment for troubleshooting purposes

        ## Navigate back to starting pose

        ## Make service call in if statement below to navigator node with desired pose as input
            ## Service would check if within tolerance and return bool
            ##      If within tolerance, stop and return true
            ##      Else, set wheel speeds for go to pose and return false

        pose = self.get_curr_pose()
        print(pose.x)
        print(pose.y)

        if(self.goal_set):
            resp = self.nav_to_pose(self.goal_pose.x, self.goal_pose.y, self.goal_pose.th)
            if(resp.reachedGoal):
                print("Arrived at destination!")
                newState = "end"
            else:
                newState = "phase3"
        else:
            newState = "phase3"

        return newState



    def run(self):
        sm = StateMachine()
        sm.add_state("idle", self.Idle)
        sm.add_state("localize", self.Localize)
        sm.add_state("phase3", self.PhaseThree)
        sm.add_state("end", None, end_state = 1)
        sm.set_start("idle")
        print("Start!")
        sm.run()



if __name__ == '__main__':
    Lab4_Phase3().run()
