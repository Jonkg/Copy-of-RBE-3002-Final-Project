#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from coord import Coord
from state_machine import StateMachine
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



global x 
x = 5



def Idle():
    print("Idle state!")
    newState = "phase1"
    return(newState)
    


def PhaseOne():
    global x
    print("PhaseOne state!")
    if x > 0:
        newState = "phase1"
        x = x-1
    else:
        newState = "phase2"
    return(newState)



def PhaseTwo():
    print("PhaseTwo state!")
    newState = "phase3"
    return(newState)



def PhaseThree():
    print("PhaseThree state!")
    newState = "end"
    return(newState)



sm = StateMachine()
sm.add_state("idle", Idle)
sm.add_state("phase1", PhaseOne)
sm.add_state("phase2", PhaseTwo)
sm.add_state("phase3", PhaseThree)
sm.add_state("end", None, end_state = 1)
sm.set_start("idle")
sm.run()
