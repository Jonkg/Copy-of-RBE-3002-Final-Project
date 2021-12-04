#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped

class Coord:
    def __init__(self, x, y):
        self.x = x
        self.y = y

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

    
    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        index = y * mapdata.info.width + x
        return index

    
    @staticmethod
    def index_to_grid(mapdata, index):
        """
        Returns the grid coordinate corresponding to the given index
        """
        x = index % mapdata.info.width
        y = (index - x) / mapdata.info.width
        return Coord(x,y)
        



    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        ### REQUIRED CREDIT
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        worldCoordx = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        worldCoordy = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        return(Point(worldCoordx, worldCoordy, 0))


        
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        #change the wp.x and wp.y to what they actually will be later on
        gridCoordx = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        gridCoordy = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return Coord(gridCoordx, gridCoordy)

        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        pose_list = []
        for pose in path:
            #setup header
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/odom"
            #setup pose
            pose = Pose()
            pose.position.x = path[0]
            pose.position.y = path[1]
            pose.position.z = 0
            pose.orientation.x
            pose.orientation.y
            pose.orientation.z = 0

            p = PoseStamped(h, pose)
            pose_list.append(p)

        return pose_list
    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        if (x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0):
            index = FrontierExplorer.grid_to_index(mapdata, x, y)
            value = mapdata.data[index]
            if (value == 0):
                return True
            else:
                return False
        else:
            return False

               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        index = FrontierExplorer.grid_to_index(mapdata, x, y)

        # Get index of the 4 adjacent cells
        neighbors = []
        neighbors.append(Coord(x-1,y))      # left
        neighbors.append(Coord(x+1,y))      # right
        neighbors.append(Coord(x,y-1))      # up
        neighbors.append(Coord(x,y+1))      # down

        # return list of coordinates of 4 neighbors
        return neighbors           

    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        index = FrontierExplorer.grid_to_index(mapdata, x, y)

        # Get index of the 8 adjacent cells
        neighbors = []
        neighbors.append(Coord(x-1,y))      # left
        neighbors.append(Coord(x+1,y))      # right
        neighbors.append(Coord(x,y-1))      # up
        neighbors.append(Coord(x,y+1))      # down
        neighbors.append(Coord(x-1,y-1))    # up, left
        neighbors.append(Coord(x+1,y-1))    # up, right
        neighbors.append(Coord(x-1,y+1))    # down, left
        neighbors.append(Coord(x+1,y+1))    # down, right

        # return list of coordinates of 8 neighbors
        return neighbors

    
    
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
                coord = FrontierExplorer.index_to_grid(occ_grid, cell_index)
                neighbors = FrontierExplorer.neighbors_of_8(occ_grid, coord.x, coord.y)
                for neighbor in neighbors:
                    neighbor_index = FrontierExplorer.grid_to_index(occ_grid, neighbor.x, neighbor.y)
                    if(occ_grid[neighbor_index] == -1):
                        frontier_cells.append(FrontierExplorer.grid_to_world(occ_grid, coord.x, coord.y))
                        

        pass


    def refineFrontier():
        # expand frontier_cells
        # shrink frontier_cells

        #expand and shrink are separate functions

        pass


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






    