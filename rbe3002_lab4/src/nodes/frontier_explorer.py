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
        self.frontier_centroid_pub = rospy.Publisher('/frontier_centroids', GridCells, queue_size = 10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Frontier explorer node ready")



    def publishFrontier(self, mapdata):
        rospy.loginfo("Publishing frontier")
        frontier_cell_indices = self.getFrontierCellIndices(mapdata)
        #refined_frontier_cell_indices = self.refineFrontier(mapdata, frontier_cell_indices, 1)

        ## Identify frontiers
        frontiers = self.identifyFrontiers(mapdata, frontier_cell_indices)
        frontier_centroids = []
        for frontier in frontiers:
            frontier_centroids.append(self.calc_centroid(mapdata, frontier))

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

        ## Create list of frontier 
        frontier_centroid_cells = []
        for centroid in frontier_centroids:
            frontier_centroid_cells.append(Lab4Util.grid_to_world(mapdata, centroid.x, centroid.y))

        ## Create a GridCells message and publish it
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        cell_width = mapdata.info.resolution*3
        cell_height = mapdata.info.resolution*3
        centroid_gridCells = GridCells(h, cell_width, cell_height, frontier_centroid_cells)
        self.frontier_centroid_pub.publish(centroid_gridCells)


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



    def identifyFrontiers(self, mapdata, frontier_cell_indices):
        """
        Identifies the frontier
        :param mapdata [OccupancyGrid] The map data
        :return        [[[int]]] A list of lists of frontier cell indices
        """

        ## Initialize structures
        frontier_num = 1
        frontDict = {}
        dictKeyList = []
        final_frontier_list = []

        ## Iterate through all frontier cells:
        for curr_index in frontier_cell_indices:

            ## List for neighboring frontiers
            neighbor_frontier_list = []

            ## get neighbors of 8.
            curr_coord = Lab4Util.index_to_grid(mapdata, curr_index)
            neighbors = Lab4Util.neighbors_of_8(mapdata, curr_coord.x, curr_coord.y)

            ## For each neighbor
            for neighbor in neighbors:

                ## Get neighbor index
                neighbor_index = Lab4Util.grid_to_index(mapdata, neighbor.x, neighbor.y)

                ## For key in key list:
                for key in dictKeyList:

                    ## For cell_index in dict[key]:
                    for cell_index in frontDict[key]:

                        ## if cell_index == neighbor_index:
                        if cell_index == neighbor_index and key not in neighbor_frontier_list:

                            ## add key to neighboring frontiers list
                            neighbor_frontier_list.append(key)

            ## if no neighboring frontiers (create new)
            if len(neighbor_frontier_list) == 0:

                ## create new frontier, add to key list, increment counter
                frontDict[frontier_num] = [curr_index]
                dictKeyList.append(frontier_num)
                frontier_num = frontier_num + 1

            ## if 1 neighboring frontier (join)
            elif len(neighbor_frontier_list) == 1:

                curr_list = frontDict[neighbor_frontier_list[0]]
                (frontDict[neighbor_frontier_list[0]])
                ## set dict[key] to list of previous list plus new cell index
                rospy.loginfo(frontDict[neighbor_frontier_list[0]])
                frontDict[neighbor_frontier_list[0]].append(curr_index)
                rospy.loginfo(frontDict[neighbor_frontier_list[0]])

            ## if 2+ neighboring frontiers (merge)
            else:

                ## create new list and add current cell index
                new_frontier_list = []
                new_frontier_list.append(curr_index)

                ## Add all cell indices from neighboring frontiers, and take neighboring frontiers off list
                rospy.loginfo(neighbor_frontier_list)
                for key in neighbor_frontier_list:
                    for cell_index in frontDict[key]:
                        new_frontier_list.append(cell_index)
                    dictKeyList.remove(key)
                
                ## create new frontier, add to key list, increment counter
                frontDict[frontier_num] = new_frontier_list
                dictKeyList.append(frontier_num)
                frontier_num = frontier_num + 1

        ## Create final list of frontiers
        for key in dictKeyList:
            if len(frontDict[key]) > 2:
                final_frontier_list.append(frontDict[key])

        ## return list of frontiers (lists of frontier cell indices)
        return final_frontier_list




    def getBestFrontier(self, mapdata):
        # for frontier in frontier_list
        #   calc_centroid
        #   calc_value
        #   add to priority queue
        

        pass


    def calc_centroid(self, mapdata, cell_list):
        x_sum = 0
        y_sum = 0

        for cell_index in cell_list:
            curr = Lab4Util.index_to_grid(mapdata, cell_index)
            x_sum = x_sum + curr.x
            y_sum = y_sum + curr.y
        
        return Coord(x_sum/len(cell_list), y_sum/len(cell_list))


    def calc_value():

        pass



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    FrontierExplorer().run()




    