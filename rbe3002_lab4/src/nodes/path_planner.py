#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from coord import Coord
from lab4_util import Lab4Util
from priority_queue import PriorityQueue
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



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

    
    
    def get_cspace_map(self):
        try: 
            get_cspace = rospy.ServiceProxy('get_cspace', GetMap)
            resp = get_cspace()
            return resp.map
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)


    
    def a_star(self, mapdata, start, goal):
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start.x, start.y, goal.x, goal.y))

        ## Publish GridCells msg with goal cell
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "/map"
        self.goal_pub.publish(GridCells(h, mapdata.info.resolution, mapdata.info.resolution, [Lab4Util.grid_to_world(mapdata, goal.x, goal.y)]))

        ## Lists to keep track of frontier, visited cells, and path
        frontier_indices = []
        expanded_indices = []
        path = []

        ## Initialize priority queue and add start cell to frontier
        frontier = PriorityQueue()
        frontier.put(start, 0)
        frontier_indices.append(Lab4Util.grid_to_index(mapdata, start.x, start.y))

        ## "Came From" and "Cost" lists for A* Algorithm
        came_from = {}
        cost_so_far = {}

        start_index = Lab4Util.grid_to_index(mapdata, start.x, start.y)
        cost_so_far[start_index] = 0
        came_from[start_index] = None

        ## Run A* Algorithm
        while not frontier.empty():

            current = frontier.get()
            curr_index = Lab4Util.grid_to_index(mapdata, current.x, current.y)
            frontier_indices.remove(curr_index)
            expanded_indices.append(curr_index)

            ## Finished when current cell is the goal
            if (current.x == goal.x and current.y == goal.y):
                while(True):
                    path.insert(0, current)
                    previous = came_from[Lab4Util.grid_to_index(mapdata, current.x, current.y)]
                    if (previous == None):
                        break
                    current = previous
                break

            ## Evaluate neighbors of current cell
            for neighbor in Lab4Util.neighbors_of_8(mapdata, current.x, current.y):
                neighbor_index = Lab4Util.grid_to_index(mapdata, neighbor.x, neighbor.y)

                ## Check if neighbors are navigable
                if (Lab4Util.is_cell_walkable(mapdata, neighbor.x, neighbor.y)):

                    ## Double cost if bordering obstacle
                    multiplier = 1
                    for neighbors_neighbor in Lab4Util.neighbors_of_8(mapdata, neighbor.x, neighbor.y):
                        neighbors_neighbor_index = Lab4Util.grid_to_index(mapdata, neighbors_neighbor.x, neighbors_neighbor.y)
                        if (not Lab4Util.is_cell_walkable(mapdata, neighbors_neighbor.x, neighbors_neighbor.y)):
                            multiplier = 2

                    new_cost = cost_so_far[curr_index] + Lab4Util.euclidean_distance(neighbor.x, neighbor.y, current.x, current.y) * multiplier

                    ## Add to frontier if previously undiscovered or cheaper
                    if (neighbor_index not in cost_so_far or new_cost < cost_so_far[neighbor_index]):
                        cost_so_far[neighbor_index] = new_cost

                        priority = new_cost + Lab4Util.euclidean_distance(neighbor.x, neighbor.y, goal.x, goal.y)

                        frontier.put(neighbor, priority)
                        frontier_indices.append(Lab4Util.grid_to_index(mapdata, neighbor.x, neighbor.y))
                        came_from[neighbor_index] = current

            ## Publish GridCells msg with current frontier
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            frontier_cells = []
            for i in frontier_indices:
                coord = Lab4Util.index_to_grid(mapdata, i)
                frontier_cells.append(Lab4Util.grid_to_world(mapdata, coord.x, coord.y))
            frontier_grid_cells = GridCells(h, mapdata.info.resolution, mapdata.info.resolution, frontier_cells)
            self.frontier_pub.publish(frontier_grid_cells)

            ## Publish GridCells msg with expanded cells
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "/map"
            expanded_cells = []
            for i in expanded_indices:
                coord = Lab4Util.index_to_grid(mapdata, i)
                expanded_cells.append(Lab4Util.grid_to_world(mapdata, coord.x, coord.y))
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
            worldCoord = Lab4Util.grid_to_world(mapdata, coord.x, coord.y)
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
        mapdata = self.get_cspace_map()
        if mapdata is None:
            rospy.loginfo("Error getting map")
            return None

        ## Execute A*
        start = Lab4Util.world_to_grid(mapdata, Coord(msg.start.pose.position.x, msg.start.pose.position.y))
        goal  = Coord(msg.goal.pose.position.x, msg.goal.pose.position.y)
        path  = self.a_star(mapdata, start, goal)

        ## Optimize path
        #waypoints = PathPlanner.optimize_path(path)

        ## Get Path message
        path_msg = self.path_to_message(mapdata, path)
        self.path_pub.publish(path_msg)
        return path_msg


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
