#!/usr/bin/env python

import rospy
import std_msgs.msg
from coord import Coord
from lab4_util import Lab4Util
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetMap, GetMapResponse
from geometry_msgs.msg import Point, Pose, PoseStamped



class CspaceCalculator:
   
    def __init__(self):
        """
        Class constructor
        """
        
        ## Store scpace map
        self.cspace_map = None

        ## Initialize node
        rospy.init_node("cspace_calculator")

        ## Subscribe to map topic
        rospy.Subscriber('/map', OccupancyGrid, self.calc_cspace)

        ## Create a Cspace service (might have to change msg type)
        get_cspace = rospy.Service('get_cspace', GetMap, self.get_cspace)
        
        ## Create C-space publisher
        self.cspace_pub = rospy.Publisher('/cspace', GridCells, queue_size = 10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1)
        rospy.loginfo("C-space calculator node ready")



    def get_cspace(self, msg):
        """
        Gets the cspace 
        """
        resp = GetMapResponse()
        resp.map = self.cspace_map
        return resp
    


    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('static_map')
        try: 
            get_map = rospy.ServiceProxy('static_map', GetMap)
            resp = get_map()
            rospy.loginfo("Got map succesfully")
            return resp.map
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)



    def calc_cspace(self, mapdata):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")
        padding = 3

        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        obstacle_cell_indices = []
        added_cell_indices = []
        added_cells = []

        ## Create new mapdata for c-space
        curr_occ_grid = mapdata
        cspace_data = []

        ## Iterate through mapdata and get list of obstacles cell indices
        ## Consider each cell
        for index in range(len(curr_occ_grid.data)):
            ## Add to list if obstacle
            if (curr_occ_grid.data[index] >= 50):
                obstacle_cell_indices.append(index)
                cspace_data.append(100)
            else:
                cspace_data.append(curr_occ_grid.data[index])

        while padding > 0:   
            ## Iterate through list of obstacle cells and list cells to add
            for cell_index in obstacle_cell_indices:
                coord = Lab4Util.index_to_grid(curr_occ_grid, cell_index)
                ## Get neighboring cells
                neighbors = Lab4Util.neighbors_of_8(curr_occ_grid, coord.x, coord.y)
                for neighbor in neighbors:
                    neighbor_index = Lab4Util.grid_to_index(curr_occ_grid, neighbor.x, neighbor.y)
                    ## Add free neighbors to added cells
                    if (curr_occ_grid.data[neighbor_index] == 0):
                        added_cell_indices.append(neighbor_index)
            ## Update obstacle cells
            for index in added_cell_indices:
                if index not in obstacle_cell_indices:
                    obstacle_cell_indices.append(index)
                    cspace_data[index] = 100
            ## Decrease padding value
            padding = padding - 1
            ## Create new Occupancy Grid
            curr_occ_grid = OccupancyGrid(mapdata.header, mapdata.info, cspace_data)

        ## Add C-space cells to list
        for index in added_cell_indices:
            coord = Lab4Util.index_to_grid(curr_occ_grid, index)
            added_cells.append(Lab4Util.grid_to_world(curr_occ_grid, coord.x, coord.y))

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
        c_space = OccupancyGrid(h, mapdata.info, curr_occ_grid.data)
        self.cspace_map = c_space
        return mapdata



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    CspaceCalculator().run()
