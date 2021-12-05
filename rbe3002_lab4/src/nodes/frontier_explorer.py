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
        rospy.Subscriber('/map', OccupancyGrid, self.publishFrontier)
        
        ## Frontier Publisher
        self.frontier_pub = rospy.Publisher('/frontier', GridCells, queue_size = 10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Frontier explorer node ready")



    def publishFrontier(self, mapdata):
        rospy.loginfo("Calculating C-Space")
        frontier_cell_indices = self.getFrontierCellIndices(mapdata)
        #refined_frontier_cell_indices = self.refineFrontier(mapdata, frontier_cell_indices, 2)

        ## Create list of frontier world cells
        frontier_cells = []
        for index in frontier_cell_indices:
            coord = Lab4Util.index_to_grid(mapdata, index)
            frontier_cells.append(Lab4Util.grid_to_world(mapdata, coord.x, coord.y))

        ## Create a GridCells message and publish it
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        cell_width = mapdata.info.resolution
        cell_height = mapdata.info.resolution
        frontier_gridCells = GridCells(h, cell_width, cell_height, frontier_cells)
        self.frontier_pub.publish(frontier_gridCells)


    def dilateFrontiers(self, mapdata, frontier_list, padding):
        """
        Refines the frontiers
        :param mapdata [OccupancyGrid]  The map data
        :param padding [int]            Number of cells to expand by
        :return        [[int]]          A refined list of frontier cell indices.
        """
        ## Add layer(s) of cells to frontier
        added_cell_indices = []

        while padding > 0:   
            ## Iterate through list of frontier cells and add walkable neighbors to a new list
            for index in frontier_list:
                coord = Lab4Util.index_to_grid(mapdata, index)
                ## Get neighboring cells
                neighbors = Lab4Util.neighbors_of_8(mapdata, coord.x, coord.y)
                for neighbor in neighbors:
                    neighbor_index = Lab4Util.grid_to_index(mapdata, neighbor.x, neighbor.y)
                    ## Add free neighbors to added cells list
                    if (neighbor_index not in frontier_list and mapdata.data[neighbor_index] == 0):
                        added_cell_indices.append(neighbor_index)
            ## Update frontier list
            for index in added_cell_indices:
                frontier_list.append(index)
            ## Decrease padding value
            padding = padding - 1

        return frontier_list



    def erodeFrontiers(self, mapdata, frontier_list, padding):
        """
        Refines the frontiers
        :param mapdata [OccupancyGrid]  The map data
        :param padding [int]            Number of cells to shrink by
        :return        [[int]]          A refined list of frontier cell indices.
        """
        ## Add layer(s) of cells to frontier
        reduced_frontier_indices = []

        while padding > 0:   
            ## Iterate through list of frontier cells and add interior cells to new list
            for index in frontier_list:
                coord = Lab4Util.index_to_grid(mapdata, index)
                ## Get neighboring cells
                neighbors = Lab4Util.neighbors_of_8(mapdata, coord.x, coord.y)
                isInterior = True
                for neighbor in neighbors:
                    neighbor_index = Lab4Util.grid_to_index(mapdata, neighbor.x, neighbor.y)
                    ## Set isInterior false if any neighbors are not in frontier
                    if (neighbor_index not in frontier_list):
                        isInterior = False
                if isInterior:
                    reduced_frontier_indices.append(neighbor_index)
            ## Update frontier list
            frontier_list = reduced_frontier_indices
            ## Decrease padding value
            padding = padding - 1

        return frontier_list

      
 
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

        for cell_index in range(len(mapdata.data)):
            if(mapdata.data[cell_index] == 0):
                coord = Lab4Util.index_to_grid(mapdata, cell_index)
                neighbors = Lab4Util.neighbors_of_8(mapdata, coord.x, coord.y)
                isBorder = False
                for neighbor in neighbors:
                    neighbor_index = Lab4Util.grid_to_index(mapdata, neighbor.x, neighbor.y)
                    if(mapdata.data[neighbor_index] == -1):
                        isBorder = True
                if isBorder:
                    frontier_cell_indices.append(cell_index)

        return frontier_cell_indices



    def refineFrontier(self, mapdata, frontier_list, padding):
        """
        Refines the frontiers
        :param mapdata [OccupancyGrid]  The map data
        :return        [[int]]          A refined list of frontier cell indices.
        """

        ## Expand frontiers
        dilatedFrontier = self.dilateFrontiers(mapdata, frontier_list, padding)

        ## Shrink frontiers
        refinedFrontier = self.erodeFrontiers(mapdata, dilatedFrontier, padding)

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




    