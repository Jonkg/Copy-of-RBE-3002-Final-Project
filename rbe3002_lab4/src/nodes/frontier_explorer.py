#!/usr/bin/env python

import rospy
import std_msgs.msg
from coord import Coord
from lab4_util import Lab4Util
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



class FrontierExplorer:

    def __init__(self):
        """
        Class constructor
        """
        ## Initialize node
        rospy.init_node("frontier_explorer")

        ## Subscribe to map topic
        rospy.Subscriber('/map', OccupancyGrid, self.something)
        
        ## Create C-space publisher
        self.cspace_pub = rospy.Publisher('/frontier', GridCells, queue_size = 10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("C-space calculator node ready")



    def dilate_frontiers(self, mapdata, padding):
        """
        Refines the frontiers
        :param mapdata [OccupancyGrid]  The map data
        :param padding [int]            Number of cells to expand by
        :return        [[int]]          A refined list of frontier cell indices.
        """
        
        return

    def erode_frontiers(self, mapdata, padding):
        """
        Refines the frontiers
        :param mapdata [OccupancyGrid]  The map data
        :param padding [int]            Number of cells to shrink by
        :return        [[int]]          A refined list of frontier cell indices.
        """

        return



    def getFrontierCell(self, mapdata):
        """
        Gets the frontier cell
        :param mapdata [OccupancyGrid] The map data
        """
        # for each cell:
        #   if(cell = 0):
        #       neighbors_of_8
        #       for neightbors:
        #           if(-1):
        #               add cell to frontier_cells

        frontier_cells = []

        occ_grid = mapdata

        for cell_index in range(len(occ_grid)):
            if(occ_grid[cell_index] == 0):
                coord = Lab4Util.index_to_grid(occ_grid, cell_index)
                neighbors = Lab4Util.neighbors_of_8(occ_grid, coord.x, coord.y)
                for neighbor in neighbors:
                    neighbor_index = Lab4Util.grid_to_index(occ_grid, neighbor.x, neighbor.y)
                    if(occ_grid[neighbor_index] == -1):
                        frontier_cells.append(Lab4Util.grid_to_world(occ_grid, coord.x, coord.y))
                        

        pass


    def refineFrontier(self, mapdata):
        """
        Refines the frontiers
        :param mapdata [OccupancyGrid]  The map data
        :return        [[int]]          A refined list of frontier cell indices.
        """

        ## Expand frontiers
        dilatedFrontier = FrontierExplorer.dilate_frontiers(mapdata)

        ## Shrink frontiers
        refinedFrontier = FrontierExplorer.erode_frontiers(dilatedFrontier)

        ## Return refined frontier
        return refinedFrontier



    def identifyFrontier():
        # for frontier cell:
        #   neighbors_of_8
        #       for frontier in frontier_list:
        #           for cell in frontier:
        #               if neighbor in a frontier:
        #                   add cell to that frontier
        #               else
        #                   create new frontier
        #  main_list[fronter1, frontier2, ...]
        #   mainlist.append([cell])

        pass


    def getBestFrontier():
        # for frontier in frontier_list
        #   calc_centroid
        #   calc_value
        #   add to priority queue

        pass

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    FrontierExplorer().run()




    