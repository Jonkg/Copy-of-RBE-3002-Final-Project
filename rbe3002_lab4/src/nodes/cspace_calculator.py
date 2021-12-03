#!/usr/bin/env python

import rospy
import std_msgs.msg
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



class Coord:
    def __init__(self, x, y):
        self.x = x
        self.y = y



class CspaceCalculator:


    
    def __init__(self):
        """
        Class constructor
        """
        ## Initialize node
        rospy.init_node("cspace_calculator")

        ## Subscribe to map topic
        rospy.Subscriber('/map', OccupancyGrid, self.calc_cspace)
        
        ## Create C-space publisher
        self.cspace_pub = rospy.Publisher('/cspace', GridCells, queue_size = 10)

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
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        index = CspaceCalculator.grid_to_index(mapdata, x, y)

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

    

    def calc_cspace(self, mapdata):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")
        padding = 6

        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        cspace_data = []
        added_cells = []

        ## Create new mapdata for c-space
        curr_occ_gri = mapdata

        ## Add 1 layer of padding per loop iteration
        while (padding > 0):
            ## Iterate through every cell in the map
            for cell_index in range(len(curr_occ_gri.data)):
                ## Consider if each free cell should become an obstacle
                if (curr_occ_gri.data[cell_index] == 0):
                    coord = CspaceCalculator.index_to_grid(curr_occ_gri, cell_index)
                    neighbors = CspaceCalculator.neighbors_of_8(curr_occ_gri, coord.x, coord.y)
                    ## Check if any neighbors are obstacles
                    adjacentToObstacle = False
                    for neighbor in neighbors:
                        neighbor_index = CspaceCalculator.grid_to_index(curr_occ_gri, neighbor.x, neighbor.y)
                        if (curr_occ_gri.data[neighbor_index] != 0):
                            adjacentToObstacle = True
                    ## If any neighbors are obsactles, make cell an obstacle in new mapdata
                    if (adjacentToObstacle):
                        cspace_data.append(100)
                        added_cells.append(CspaceCalculator.grid_to_world(curr_occ_gri, coord.x, coord.y))
                    ## Otherwise, keep the cell free in new mapdata
                    else:
                        cspace_data.append(0)
                ## If already obstacle, mark as obstacle in new mapdata
                else:
                    cspace_data.append(100)  
            padding = padding - 1
            ## Create new occupancy grid from new mapdata for c-sapce
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



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    CspaceCalculator().run()
