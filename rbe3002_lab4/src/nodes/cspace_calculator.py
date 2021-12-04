#!/usr/bin/env python

import rospy
import std_msgs.msg
from coord import Coord
from lab4_util import Lab4Util
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



class CspaceCalculator:


    
    def __init__(self):
        """
        Class constructor
        """
        ## Initialize node
        rospy.init_node("cspace_calculator")

        ## Subscribe to map topic
        rospy.Subscriber('/map', OccupancyGrid, self.calc_cspace)
        
        ## Create C-space publisher
        self.cspace_pub = rospy.Publisher('/cspace', GridCells, queue_size = 10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("C-space calculator node ready")

    

    def calc_cspace(self, mapdata):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")
        padding = 6

        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        cspace_data = []
        added_cells = []

        ## Create new mapdata for c-space
        curr_occ_gri = mapdata

        ## Add 1 layer of padding per loop iteration
        while (padding > 0):
            ## Iterate through every cell in the map
            for cell_index in range(len(curr_occ_gri.data)):
                ## Consider if each free cell should become an obstacle
                if (curr_occ_gri.data[cell_index] == 0):
                    coord = Lab4Util.index_to_grid(curr_occ_gri, cell_index)
                    neighbors = Lab4Util.neighbors_of_8(curr_occ_gri, coord.x, coord.y)
                    ## Check if any neighbors are obstacles
                    adjacentToObstacle = False
                    for neighbor in neighbors:
                        neighbor_index = Lab4Util.grid_to_index(curr_occ_gri, neighbor.x, neighbor.y)
                        if (curr_occ_gri.data[neighbor_index] != 0):
                            adjacentToObstacle = True
                    ## If any neighbors are obsactles, make cell an obstacle in new mapdata
                    if (adjacentToObstacle):
                        cspace_data.append(100)
                        added_cells.append(Lab4Util.grid_to_world(curr_occ_gri, coord.x, coord.y))
                    ## Otherwise, keep the cell free in new mapdata
                    else:
                        cspace_data.append(0)
                ## If already obstacle, mark as obstacle in new mapdata
                else:
                    cspace_data.append(100)  
            padding = padding - 1
            ## Create new occupancy grid from new mapdata for c-sapce
            curr_occ_gri = OccupancyGrid(mapdata.header, mapdata.info, cspace_data)

        ## Create a GridCells message and publish it
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        cell_width = mapdata.info.resolution
        cell_height = mapdata.info.resolution
        c_space_obstacles = GridCells(h, cell_width, cell_height, added_cells)
        self.cspace_pub.publish(c_space_obstacles)

        ## Return the C-space
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        c_space = OccupancyGrid(h, mapdata.info, cspace_data)
        return c_space



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    CspaceCalculator().run()
