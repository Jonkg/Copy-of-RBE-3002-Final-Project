#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from coord import Coord
from priority_queue import PriorityQueue
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped

global map



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
        s = rospy.Service('plan_path', GetPlan, self.plan_path)
        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace_pub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size = 10)
        
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose the topic names, the message type is GridCells
        self.expanded_pub = rospy.Publisher('/path_planner/expanded', GridCells, queue_size = 10)
        self.frontier_pub = rospy.Publisher('/path_planner/frontier', GridCells, queue_size = 10)
        self.goal_pub = rospy.Publisher('/path_planner/goal', GridCells, queue_size = 10)

        ## Path publisher
        self.path_pub = rospy.Publisher('/path_planner/path_pub', Path, queue_size = 10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1)
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
            index = PathPlanner.grid_to_index(mapdata, x, y)
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
        index = PathPlanner.grid_to_index(mapdata, x, y)

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
        index = PathPlanner.grid_to_index(mapdata, x, y)

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
        cspace_data = []
        added_cells = []

        curr_occ_gri = mapdata

        while (padding > 0):
            for cell_index in range(len(curr_occ_gri.data)):
                if (curr_occ_gri.data[cell_index] == 0):
                    coord = PathPlanner.index_to_grid(curr_occ_gri, cell_index)
                    neighbors = PathPlanner.neighbors_of_8(curr_occ_gri, coord.x, coord.y)
                    adjacentToObstacle = False
                    for neighbor in neighbors:
                        neighbor_index = PathPlanner.grid_to_index(curr_occ_gri, neighbor.x, neighbor.y)
                        if (curr_occ_gri.data[neighbor_index] != 0):
                            adjacentToObstacle = True
                    if (adjacentToObstacle):
                        cspace_data.append(100)
                        added_cells.append(PathPlanner.grid_to_world(curr_occ_gri, coord.x, coord.y))
                    else:
                        cspace_data.append(0)
                else:
                    cspace_data.append(100)  
            padding = padding - 1
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


    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start.x, start.y, goal.x, goal.y))

        ## Publish GridCells msg with goal cell
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        self.goal_pub.publish(GridCells(h, mapdata.info.resolution, mapdata.info.resolution, [PathPlanner.grid_to_world(mapdata, goal.x, goal.y)]))

        ## Lists to keep track of frontier, visited cells, and path
        frontier_indices = []
        expanded_indices = []
        path = []

        ## Initialize priority queue and add start cell to frontier
        frontier = PriorityQueue()
        frontier.put(start, 0)
        frontier_indices.append(PathPlanner.grid_to_index(mapdata, start.x, start.y))

        ## "Came From" and "Cost" lists for A* Algorithm
        came_from = {}
        cost_so_far = {}

        start_index = PathPlanner.grid_to_index(mapdata, start.x, start.y)
        cost_so_far[start_index] = 0
        came_from[start_index] = None

        ## Run A* Algorithm
        while not frontier.empty():

            current = frontier.get()
            curr_index = PathPlanner.grid_to_index(mapdata, current.x, current.y)
            frontier_indices.remove(curr_index)
            expanded_indices.append(curr_index)

            ## Finished when current cell is the goal
            if (current.x == goal.x and current.y == goal.y):
                while(True):
                    path.insert(0, current)
                    previous = came_from[PathPlanner.grid_to_index(mapdata, current.x, current.y)]
                    if (previous == None):
                        break
                    current = previous
                break

            ## Evaluate neighbors of current cell
            for neighbor in PathPlanner.neighbors_of_8(mapdata, current.x, current.y):
                neighbor_index = PathPlanner.grid_to_index(mapdata, neighbor.x, neighbor.y)

                ## Check if neighbors are navigable
                if (PathPlanner.is_cell_walkable(mapdata, neighbor.x, neighbor.y)):
                    new_cost = cost_so_far[curr_index] + PathPlanner.euclidean_distance(neighbor.x, neighbor.y, current.x, current.y)

                    ## Add to frontier if previously undiscovered or cheaper
                    if (neighbor_index not in cost_so_far or new_cost < cost_so_far[neighbor_index]):
                        cost_so_far[neighbor_index] = new_cost

                        priority = new_cost + PathPlanner.euclidean_distance(neighbor.x, neighbor.y, goal.x, goal.y)

                        frontier.put(neighbor, priority)
                        frontier_indices.append(PathPlanner.grid_to_index(mapdata, neighbor.x, neighbor.y))
                        came_from[neighbor_index] = current

            ## Publish GridCells msg with current frontier
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            frontier_cells = []
            for i in frontier_indices:
                coord = PathPlanner.index_to_grid(mapdata, i)
                frontier_cells.append(PathPlanner.grid_to_world(mapdata, coord.x, coord.y))
            frontier_grid_cells = GridCells(h, mapdata.info.resolution, mapdata.info.resolution, frontier_cells)
            self.frontier_pub.publish(frontier_grid_cells)

            ## Publish GridCells msg with expanded cells
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            expanded_cells = []
            for i in expanded_indices:
                coord = PathPlanner.index_to_grid(mapdata, i)
                expanded_cells.append(PathPlanner.grid_to_world(mapdata, coord.x, coord.y))
            expanded_grid_cells = GridCells(h, mapdata.info.resolution, mapdata.info.resolution, expanded_cells)
            self.expanded_pub.publish(expanded_grid_cells)
        
        return path

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        ## Create
        optimal_path = []

        ## return input path is less than 3 points
        if (len(path) < 3):
            return path

        ## Add first point to optimal path
        optimal_path.append(path[0])
        prev = path[0]
        curr = path[1]

        ## Add direction change points to path
        for i in range(2,len(path)):
            next = path[i]
            prev_delta = Coord(curr.x-prev.x, curr.y-prev.y)
            next_delta = Coord(next.x-curr.x, next.y-curr.y)
            if ((prev_delta.x != next_delta.x) or (prev_delta.y != next_delta.y)):
                optimal_path.append(curr)
            prev = curr
            curr = next

        ## Add last point to path
        optimal_path.append(next)

        ## Return path
        rospy.loginfo(optimal_path)
        return optimal_path

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        pose_list = []

        for coord in path:
            pose = Pose()
            worldCoord = PathPlanner.grid_to_world(mapdata, coord.x, coord.y)
            pose.position.x = worldCoord.x
            pose.position.y = worldCoord.y
            pose.position.z = worldCoord.z

            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            pose_stamped = PoseStamped(h, pose)
            pose_list.append(pose_stamped)

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        path_msg = Path(h, pose_list)

        return path_msg


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        map = PathPlanner.request_map()
        if map is None:
            return Path()

        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(map, 1)

        ## Execute A*
        start = PathPlanner.world_to_grid(map, Coord(msg.start.pose.position.x, msg.start.pose.position.y))
        goal  = PathPlanner.world_to_grid(map, Coord(msg.goal.pose.position.x, msg.goal.pose.position.y))
        path  = self.a_star(cspacedata, start, goal)

        ## Optimize path
        waypoints = PathPlanner.optimize_path(path)

        ## Get Path message
        path_msg = self.path_to_message(map, waypoints)
        self.path_pub.publish(path_msg)
        return path_msg


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
