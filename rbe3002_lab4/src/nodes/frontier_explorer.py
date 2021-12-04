#!/usr/bin/env python

from codecs import ignore_errors
import rospy
from rospy.core import add_shutdown_hook
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

      
 
    def getFrontierCellIndices(self, mapdata):

        """
        Gets frontier cell indices from the given map data
        :param mapdata [OccupancyGrid] The map data
        :return        [[int]] The list of indices of the frontier cells
        """
        # for each cell:
        #   if(cell = 0):
        #       neighbors_of_8
        #       for neightbors:
        #           if(-1):
        #               add cell to frontier_cells

        frontier_cell_indices = []

        occ_grid = mapdata

        for cell_index in range(len(occ_grid)):
            if(occ_grid[cell_index] == 0):
                coord = Lab4Util.index_to_grid(occ_grid, cell_index)
                neighbors = Lab4Util.neighbors_of_8(occ_grid, coord.x, coord.y)
                for neighbor in neighbors:
                    neighbor_index = Lab4Util.grid_to_index(occ_grid, neighbor.x, neighbor.y)
                    if(occ_grid[neighbor_index] == -1):
                        frontier_cell_indices.append(cell_index)

        return frontier_cell_indices


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



    def identifyFrontiers(self, mapdata):
        """
        Identifies the frontier
        :param mapdata [OccupancyGrid] The map data
        :return        [[[int]]] A list of lists of frontier cell indices
        """

        # for frontier cell:
        #   neighbors_of_8
        #       for frontier in frontier_list:
        #           for cell in frontier:
        #               if neighbor in a frontier:
        #                   add cell to that frontier
        #               else
        #                   create new frontier
        #                   mainlist.append([cell])
        #  main_list[fronter1, frontier2, ...]
        #  

        frontier_cell_indices = self.getFrontierCellIndices(mapdata)
        frontier_list = []
        main_list = []
        occ_grid = mapdata

        # for each frontier cell
        for cell_index in frontier_cell_indices:
            coord = Lab4Util.index_to_grid(occ_grid, cell_index)
            # find the neighbors of 8
            neighbors = Lab4Util.neighbors_of_8(occ_grid, coord.x, coord.y)
            for neighbor in neighbors:
                neighbor_index = Lab4Util.grid_to_index(occ_grid, neighbor.x, neighbor.y)
                # for each frontier in the frontier list
                for frontier in frontier_list:
                    # for each cell in a frontier
                    for occ_grid[cell_index] in frontier:
                        # if the neighbor index is in that frontier
                        if occ_grid[neighbor_index] in frontier:
                            frontier_list.append(cell_index)
                        else:
                            main_list.append([cell_index])
        
        return main_list




    def getBestFrontier(self, mapdata):
        # for frontier in frontier_list
        #   calc_centroid
        #   calc_value
        #   add to priority queue
        

        pass


    def calc_centroid():
        
        pass


    def calc_value():

        pass



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    FrontierExplorer().run()




    