#!/usr/bin/env python

import math
import heapq
import rospy
import std_msgs.msg
from coord import Coord
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



class Lab4Util:



    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
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
        #change the wp.x and wp.y to what they actually will be later on
        gridCoordx = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        gridCoordy = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return Coord(gridCoordx, gridCoordy)



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
        if (x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0):
            index = Lab4Util.grid_to_index(mapdata, x, y)
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
        index = Lab4Util.grid_to_index(mapdata, x, y)

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
        index = Lab4Util.grid_to_index(mapdata, x, y)

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