#!/usr/bin/env python

import math
from codecs import ignore_errors
import rospy
from rospy.core import add_shutdown_hook
import std_msgs.msg
from rbe3002_lab4.srv import BestFrontier, BestFrontierResponse
from priority_queue import PriorityQueue
from coord import Coord
from lab4_util import Lab4Util
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_srvs.srv._Empty import Empty



class FrontierExplorer:

    def __init__(self):
        """
        Class constructor
        """
        ## Initialize node
        rospy.init_node("frontier_explorer")

        ## Service call that accepts a posestamped message and call self.bestFrontier when 
        ## message is recieved
        best_frontier = rospy.Service('best_frontier', BestFrontier, self.bestFrontier)
        
        ## Frontier Publisher
        self.frontier_pub = rospy.Publisher('/frontier', GridCells, queue_size = 10)
        self.frontier_centroid_pub = rospy.Publisher('/frontier_centroids', GridCells, queue_size = 10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1)
        rospy.loginfo("Frontier explorer node ready")

        ## Wait for services to startup
        rospy.wait_for_service('get_cspace')



    def get_cspace_map(self):
        try: 
            get_cspace = rospy.ServiceProxy('get_cspace', GetMap)
            resp = get_cspace()
            return resp.map
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)



    def bestFrontier(self, msg):
        """
        Get the best frontier to go to
        :param msg [PoseStamped] A posestamped message
        :return     the centroid of the best frontier
        """
        rospy.loginfo("Publishing frontier")

        ## Get current robot position from msg
        robotPos = Coord(msg.x, msg.y)

        ## Request the map
        ## In case of error, return an empty path
        mapdata = self.get_cspace_map()

        ## Get a list of frontier cell indices
        frontier_cell_indices = self.getFrontierCellIndices(mapdata)
        #refined_frontier_cell_indices = self.refineFrontier(mapdata, frontier_cell_indices, 1)

        ## Identify independent frontiers from frontier list, and list centroids
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

        ## Create list of frontier centroid world cells
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
        
        # get the best frontier to go to and return it
        resp = self.getBestFrontier(robotPos, mapdata, frontiers)
        
        return resp



    def request_path(self, centroidX, centroidY, robotPos):
        """
        Requests the path from path_planner
        :return [Path] The grid if the service call was successful,
                                None in case of error.
        """
    
        rospy.loginfo("Requesting the path")
        try: 
            get_plan = rospy.ServiceProxy('plan_path', GetPlan)
            req = GetPlan()

            start_pose = Pose()
            start_pose.position.x = robotPos.x
            start_pose.position.y = robotPos.y
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            start_pose_stamped = PoseStamped(h, start_pose)

            goal_pose = Pose()
            goal_pose.position.x = centroidX
            goal_pose.position.y = centroidY
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            goal_pose_stamped = PoseStamped(h, goal_pose)

            req.start = start_pose_stamped
            req.goal = goal_pose_stamped
            req.tolerance = 0
            resp = get_plan(req.start, req.goal, req.tolerance)
            rospy.loginfo("Got path succesfully")
            return resp.plan
        except rospy.ServiceException as e:
           rospy.loginfo("Service call failed: %s"%e)



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

        frontier_cell_indices = []

        for cell_index in range(len(mapdata.data)):
            # get the neighbors of the coord in the current cell index
            if(mapdata.data[cell_index] == 0):
                coord = Lab4Util.index_to_grid(mapdata, cell_index)
                neighbors = Lab4Util.neighbors_of_8(mapdata, coord.x, coord.y)
                isBorder = False

                # for each neighbor, check if it is a border, and if so, append to the list
                for neighbor in neighbors:
                    neighbor_index = Lab4Util.grid_to_index(mapdata, neighbor.x, neighbor.y)
                    if(neighbor_index < len(mapdata.data)):
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
                frontDict[neighbor_frontier_list[0]].append(curr_index)

            ## if 2+ neighboring frontiers (merge)
            else:

                ## create new list and add current cell index
                new_frontier_list = []
                new_frontier_list.append(curr_index)

                ## Add all cell indices from neighboring frontiers, and take neighboring frontiers off list
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




    def getBestFrontier(self, robotPos, mapdata, frontier_list):
        """
        Gets the best frontier from a list of frontiers and returns it
        :param mapdata [OccupancyGrid] The map data
        """

        frontier_queue = PriorityQueue()
        
        # for each frontier in the list, get its weight and put it in the frontier queue
        for frontier in frontier_list:
            weight = self.calc_value(robotPos, mapdata, frontier)
            frontier_queue.put(frontier, weight)

        resp = BestFrontierResponse()

        ## Get the centroid of the best frontier if a frontier exists
        while not frontier_queue.empty():
            best_frontier = frontier_queue.get()
            centroid = self.calc_centroid(mapdata, best_frontier)
            path = self.request_path(centroid.x, centroid.y, robotPos)
            if(path != None):
                if(len(path.poses) > 0):
                    resp.path = path
                    resp.exists = True
                    return resp

        ## Return false if no frontiers with valid paths
        resp.exists = False
        
        return resp


    
    #@staticmethod
    def calc_value(self, robotPos, mapdata, frontier):
        """
        Calculates the value of a frontier, where the value is the weight of the frontier
        :param mapdata [OccupancyGrid] The map data
        :return         int  The value of the frontier
        """
        
        length = self.calc_length(mapdata, frontier)
        centroid = self.calc_centroid(mapdata, frontier)

        robotPosGC = Lab4Util.world_to_grid(mapdata, Point(robotPos.x, robotPos.y, 0))

        # distance is the euclidean distance from the current robot position to the centroid
        distance = Lab4Util.euclidean_distance(robotPosGC.x, robotPosGC.y, centroid.x, centroid.y)

        # value is the length of the frontier divided by the distance 
        # value = length/(distance**2)
        value = 1/distance

        return value



    def calc_centroid(self, mapdata, cell_list):
        """
        Caclulates the centroid of the frontier and returns the point of the frontier closest to that centroid
        :param mapdata [OccupancyGrid] The map data
        :param cell_list [int] list of cells
        return          [Coord] the coordinate of the closest frontier cell to centroid
        """
        x_sum = 0
        y_sum = 0

        # go through the frontier and add all the x's and y's to calc the centroid
        for cell_index in cell_list:
            curr = Lab4Util.index_to_grid(mapdata, cell_index)
            x_sum = x_sum + curr.x
            y_sum = y_sum + curr.y
        
        centroid = Coord(x_sum/len(cell_list), y_sum/len(cell_list))

        # get the first coordinate in the frontier and instantiate the closest distance
        first_coord = Lab4Util.index_to_grid(mapdata, cell_list[0])
        closest_cell = first_coord
        closest_distance = Lab4Util.euclidean_distance(centroid.x, centroid.y, first_coord.x, first_coord.y)

        for index in cell_list:
            # get the current coordinate and calculate the distance
            current = Lab4Util.index_to_grid(mapdata, index)
            if Lab4Util.is_cell_walkable(mapdata, current.x, current.y):
                curr_distance = Lab4Util.euclidean_distance(centroid.x, centroid.y, current.x, current.y)
                # if the current distance is less than the closest distance, set the current as the new closest distance 
                if curr_distance < closest_distance:
                    closest_distance = curr_distance
                    closest_cell = current
        
        # return the coordinate with the closest distance
        return closest_cell



    @staticmethod
    def calc_length(mapdata, frontier):
        """
        Calculates the length of the given frontier
        :param mapdata  [OccupancyGrid] The map data
        :param frontier [int]           List of cell indices in a frontier
        :return         float           The length of the frontier
        """
        # instantiate the max and mins of x and y from the first coordinate in the frontier
        first_coord = Lab4Util.index_to_grid(mapdata, frontier[0])
        max_x = first_coord.x
        min_x = first_coord.x
        max_y = first_coord.y
        min_y = first_coord.y
        
        for index in frontier:
            # get the coordinate for the current index
            current = Lab4Util.index_to_grid(mapdata, index)
            
            # if the current x coord is greater than the max x coord, set the current to the new max_x
            # else, if the current x is less than the min x, set that to the new min x 
            if current.x > max_x:
                max_x = current.x
            elif current.x < min_x:
                min_x = current.x

            # same thing as x, but for the y coord
            if current.y > max_y:
                max_y = current.y
            elif current.y < min_y:
                min_y = current.y
        
        # calculate the deltas by subtracting the max and min
        delta_x = max_x - min_x
        delta_y = max_y - min_y
        
        # calculate the length by doing trig
        length = math.sqrt(delta_x**2 + delta_y**2)
        return length



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    FrontierExplorer().run()




    