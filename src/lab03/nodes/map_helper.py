#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose


def get_neighbors(loc, my_map):
    """
        returns the legal neighbors of loc
        :param loc: tuple of location
        :return: list of tuples
    """
    #Check Neighbors (the points that are directly above, below, left, and right)
    step = round(my_map.info.resolution, 2)
    point1 = (round(loc[0], 2), round(loc[1] + step, 2),0)
    point2=(round(loc[0] + step,2), round(loc[1], 2),0)
    point3=(round(loc[0],2), round(loc[1] - step, 2),0)
    point4=(round(loc[0] - step,2), round(loc[1], 2),0)
    
    #Add valid points to the validNeighbor list
    validNeighbors = list()
    
    if is_valid_loc(point1, my_map):
        validNeighbor.append(point1)
    if is_valid_loc(point2, my_map):
        validNeighbor.append(point2)
    if is_valid_loc(point3, my_map):
        validNeighbor.append(point3)
    if is_valid_loc(point4, my_map):
        validNeighbor.append(point4)
        
    returns validNeighbors
    
def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """
    loc_x = loc[0]
    loc_y = loc[1]
    
    #Index of map found based on tuple location
    map_point = world_to_map(loc_x, loc_y, my_map)
    index = point_to_index(map_point, my_map)

    index = int(index)
    
    #Calculating map limits
    mapLimit = (my_map.info.width * my_map.info.resolution) / 2.0
    mapLimit = round(mapLimit, 2)
    negMapLimit = mapLimit * (-1)
    
    #Point is legal location if within the map & unoccupied 
    if (my_map.data[index]==0 and
        loc_x < mapLimit and loc_x > negMapLimit
        and loc_y < mapLimit and loc_y > negMapLimit):
        return True
    return False


def convert_location(loc, my_map):
    """converts points to the grid"""
   

def world_to_map(x, y, my_map):
    """
        converts a point from the world to the map
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """

def map_to_world(x, y, my_map):
    """
        converts a point from the map to the world
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """


def to_cells(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def to_poses(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def index_to_point(point, my_map):
    """convert a point to a index"""

def point_to_index(location, my_map):
    """convert a index to a point"""
