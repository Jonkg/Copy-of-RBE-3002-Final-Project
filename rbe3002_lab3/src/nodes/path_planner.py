#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap, GetMapResponse
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped

import sys
sys.path.append("~/catkin_ws/src/rbe3002_lab3/src/nodes/priority_queue.py")
import priority_queue

global map

class Coord:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        # TODO
        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        # TODO
        rospy.publisher('/path_planner/cspace', GridCells, queue_size = 10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose the topic names, the message type is GridCells
        expanded_pub = rospy.Publisher('/path_planner/expanded', GridCells, queue_size = 10)
        fronteir_pub = rospy.Publisher('/path_planner/fronteir', GridCells, queue_size = 10)
        unexplored_pub = rospy.Publisher('/apath_planner/unexplored', GridCells, queue_size = 10)
        
        ## Initialize the request counter
        # TODO
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")



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
        distance = sqrt((x2 - x1)**2 + (y2 - y1)**2)
        


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
        worldCoordx = (x + 0.5) * mapdata.info.resolution + mapdata.info.position.x
        worldCoordy = (y + 0.5) * mapdata.info.resolution + mapdata.info.position.y


        
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


        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        pass

    

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
        value = PathPlanner.grid_to_index(mapdata, x, y)
        if(value == 0 and x < mapdata.info.width and x > 0 and y < mapdata.info.height and y > 0):
            return True
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
        index = PathPlanner.grid_to_index(mapdata, x, y)

        # Get index of the 4 adjacent cells
        neighbors = []
        neighbors.append(PathPlanner.index_to_grid(mapdata, index-1))                        # left
        neighbors.append(PathPlanner.index_to_grid(mapdata, index+1))                        # right
        neighbors.append(PathPlanner.index_to_grid(mapdata, index-mapdata.info.width))      # up
        neighbors.append(PathPlanner.index_to_grid(mapdata, index+mapdata.info.width))      # down

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
        index = PathPlanner.grid_to_index(mapdata, x, y)

        # Get index of the 8 adjacent cells
        neighbors = []
        neighbors.append(PathPlanner.index_to_grid(mapdata, index-1))                        # left
        neighbors.append(PathPlanner.index_to_grid(mapdata, index+1))                        # right
        neighbors.append(PathPlanner.index_to_grid(mapdata, index-mapdata.info.width))      # up
        neighbors.append(PathPlanner.index_to_grid(mapdata, index+mapdata.info.width))      # down
        neighbors.append(PathPlanner.index_to_grid(mapdata, index-mapdata.info.width-1))    # up and left
        neighbors.append(PathPlanner.index_to_grid(mapdata, index-mapdata.info.width+1))    # up and right
        neighbors.append(PathPlanner.index_to_grid(mapdata, index+mapdata.info.width-1))    # down and left
        neighbors.append(PathPlanner.index_to_grid(mapdata, index+mapdata.info.width+1))    # down and right

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



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        # TODO 
        #index_to_grid
        for i in range(mapdata.info.width):
            PathPlanner.index_to_grid(mapdata, i)
            #neighbors of 8
            #is walkable
            #grid to index

        ## Create a GridCells message and publish it
        # TODO
        ## Return the C-space
        pass


    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        
        frontier = priority_queue.PriorityQueue
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            #for next in graph.neighbors(current)
                #new_cost = cost_so_far[current] + graph.cost(current, next)
                #if next not in cost_so_far or new_cost < cost_so_far[next]
                #   cost_so_far[next] = new_cost
                #   priority = new_cost + heuristic(goal, next)
                #   frontier.put(next, priority)
                #   came_from[next] = current




    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        map = PathPlanner.request_map()
        coords = PathPlanner.neighbors_of_8(map, 1, 1)
        for coord in coords:
            print("Neighbor coordinates: (%d, %d)", coord.x, coord.y)
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
