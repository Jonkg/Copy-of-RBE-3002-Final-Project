#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from KBHit import KBHit
from coord import Coord
from state_machine import StateMachine
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_srvs.srv._Empty import Empty



class Lab4:

    global initialPose
    global mapdata
    global goal_set



    def __init__(self):
        ## Initialize node
        rospy.init_node("lab4")

        ## Subscribe to '/move_base_simple/goal' topic to get final psoe for Phase 3
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_goal)

        rospy.sleep(1)



    def set_goal(self, msg):
        global goal_set

        goal_pose = msg.pose
        goal_set = True




    def Idle(self):
        global initialPose

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
        ## Get centroid of best frontier from 'frontier explorer'
        best_centroid = rospy.ServiceProxy('best_frontier', PoseStamped)
        ##      If frontiers to explore: Command 'navigator' node to drive to frontier centroid
        frontiers = rospy.ServiceProxy('get_frontiers', Empty)
        


        ##      Else: Save the map (decide where/how to do this) and change state

        newState = "phase2"     ## THIS IS TEMPORARY

        return(newState)



    def PhaseTwo(self):
        global initialPose
        print("PhaseTwo state!")    # Comment for troubleshooting purposes

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
        global goal_set
        print("PhaseThree state!")  # Comment for troubleshooting purposes

        ## Navigate to selected goal pose

        ## Make service call in if statement below to navigator node with desired pose as input
            ## Service would check if within tolerance and return bool
            ##      If within tolerance, stop and return true
            ##      Else, set wheel speeds for go to pose and return false

        goal_set = True     ## THIS IS TEMPORARY

        if(goal_set):
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
