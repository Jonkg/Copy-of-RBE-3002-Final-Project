#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from KBHit import KBHit
from coord import Coord
from rbe3002_lab4.srv import getPose
from state_machine import StateMachine
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



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

        rospy.sleep(1)



    def set_goal(self, msg):
        self.goal_pose = msg.pose
        self.goal_set = True



    def get_curr_pose(self):
        rospy.wait_for_service('get_pose')
        get_pose = rospy.ServiceProxy('get_pose', getPose)
        try:
            resp = get_pose()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp


    def Idle(self):
        ## If keyboard press: save initial pose and start Phase 1

        kb = KBHit()
        if kb.kbhit():
            ### SAVE INITIAL POSE HERE ###
           newState = "phase1"
        else:
            newState = "idle"
        kb.set_normal_term()

        return(newState)
        


    def PhaseOne(self):
        print("PhaseOne state!")    # Comment for troubleshooting purposes

        ## Explore unknown map and save map to file

        ## Get current pose from 'navigator' node
        ## Get centroid of best frontier from 'frontier_explorer'
        ##      If frontiers to explore: Command 'navigator' node to drive to frontier centroid
        ##      Else: Save the map (decide where/how to do this) and change state

        newState = "phase2"     ## THIS IS TEMPORARY
        self.initial_pose = self.get_curr_pose()

        return(newState)



    def PhaseTwo(self):
        print("PhaseTwo state!")    # Comment for troubleshooting purposes

        print("Go to:")
        print(self.initial_pose)

        ## Navigate back to starting pose

        ## Make service call in if statement below to navigator node with desired pose as input
            ## Service would check if within tolerance and return bool
            ##      If within tolerance, stop and return true
            ##      Else, set wheel speeds for go to pose and return false

        ## Change condition below to be the service call to navigator node
        if(True):
            newState = "phase3"
        else:
            newState = "phase2"
        
        return(newState)



    def PhaseThree(self):
        print("PhaseThree state!")  # Comment for troubleshooting purposes

        ## Navigate to selected goal pose

        ## Make service call in if statement below to navigator node with desired pose as input
            ## Service would check if within tolerance and return bool
            ##      If within tolerance, stop and return true
            ##      Else, set wheel speeds for go to pose and return false

        self.goal_set = True     ## THIS IS TEMPORARY

        if(self.goal_set):
            ## Change condition below to be the service call to navigator node
            if(True):
                newState = "end"
            else:
                newState = "phase3"
        else:
            newState = "phase3"

        return(newState)



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
