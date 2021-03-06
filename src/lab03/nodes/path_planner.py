#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from Queue import PriorityQueue


class PathPlanner:



    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner", anonymous=True)
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service('plan_path', GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        rospy.Subscriber('/map', OccupancyGrid, self.call_calc_cspace) #OccupancyGrid Subscriber
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.c_space_pub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
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
        index = y * mapdata.info.width + x

        return index

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
        distanceX = x2-x1
        distanceY = y2-y1

        euclideanDist = math.sqrt(distanceX**2 + distanceY**2)

        return euclideanDist


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        #World Coordinates
        worldp.x = int((x*mapdata.resolution) + mapdata.pose.x)
        worldp.y = int((y*mapdata.resolution) + mapdata.pose.y)

        return (worldp.x, worldp.y)
        #pass



    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        #Grid Coordinates
        gridp.x = int((worldp.x - mapdata.pose.x) / mapdata.resolution)
        gridp.y = int((worldp.y - mapdata.pose.y) / mapdata.resolution)

        return(gridp.x, gridp.y)



    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        
        poseArray = [];
        #rospy.loginfo("NODES IN PATH: "+str(len(path)));
        for k in path:
            rospy.loginfo("k[0]: "+str(k[0])+", k[1]:"+str(k[1]));
            worldC = PathPlanner.grid_to_world(mapdata, k[0], k[1]);
            poseStamped = PoseStamped();
            poseStamped.pose.position.x = worldC.x;
            poseStamped.pose.position.y = worldC.y;
            header = Header()
            header.frame_id = "map"
            poseStamped.header = header;
            poseArray.append(poseStamped);
   
        return path_to_poses;
    
        #pass



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
        index = self.grid_to_index(x,y)
        if mapdata.data[index] < obj and mapdata.info.width >= x and mapdata.info.height >= y:
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
        if self.is_cell_walkable(mapdata, x+1, y):
            walkFour.add((x+1, y))
        if self.is_cell_walkable(mapdata, x-1, y):
            walkFour.add((x-1, y))
        if self.is_cell_walkable(mapdata, x, y+1):
            walkFour.add((x, y+1))
        if self.is_cell_walkable(x, y-1):
            walkFour.is_cell_walkable((x, y+1))

        return walkFour



    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        if self.is_cell_walkable(mapdata, x+1, y):
            walkEight.add((x+1, y))
        if self.is_cell_walkable(mapdata, x-1, y):
            walkEight.add((x-1, y))
        if self.is_cell_walkable(mapdata, x, y+1):
            walkEight.add((x, y+1))
        if self.is_cell_walkable(x, y-1):
            walkEight.is_cell_walkable((x, y+1))

        if self.is_cell_walkable(mapdata, x+1, y-1):
            walkEight.add((x+1, y-1))
        if self.is_cell_walkable(mapdata, x-1, y-1):
            walkEight.add((x-1, y-1))
        if self.is_cell_walkable(mapdata, x+1, y+1):
            walkEight.add((x+1, y+1))
        if self.is_cell_walkable(x-1, y-1):
            walkEight.is_cell_walkable((x-1, y+1))

        return walkEight



    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/static_map')
        try:
            request_map = rospy.ServiceProxy('/static_map', GetMap)
            OccupancyGrid = request_map()
            return OccupancyGrid
        except rospy.ServiceException, e:
            
            print "Service call failed: %s"%e



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
        n = mapdata.info.width * mapdata.info.height
        inflatedGrid = [0]*n
        grid_pos = [0]*n

        ## Go through each cell in the occupancy grid
        for w in mapdata.data:
            x = w % mapdata.info.width
            y = (w - x)/mapdata.info.width
            grid_pos[w] = [x,y,0]
            self.edge_check(x,y,mapdata.info.width,mapdata.info.height,mapdata)
            yup1 = ((y+1)*mapdata.info.width)+x
            ydwn1 = ((y-1)*mapdata.info.width)+x
            if self.mapdata.data[w] >= objs:
                inflatedGrid[w] = 100
            elif ((not edge_check.leftMost and self.mapdata.data[w-1] >= objs) or (not edge_check.rightMost and self.mapdata.data[w+1] >= objs)
                or (not edge_check.top and self.mapdata.data[yup] >= objs) or (edge_check.bottom and self.mapdata.data[ydwn] >= objs)):
                inflatedGrid[w] = 100
        ## Inflate the obstacles where necessary
        # TODO
        ## Create a GridCells message and publish it
        msg_grid_cell = GridCells()
        # Fix header
        msg.grid_cell.Header.frame_id = "/map"
        msg_grid_cell.height = mapdata.info.height
        msg_grid_cell.width = mapdata.info.width
        msg_grid_cell.Point = grid_pos
        self.pub.publish(msg_grid_cell)
        # TODO
        ## Return the C-space
        #inflatedb grid
        return inflatedGrid
        #Go through all original map data and check for 100, if 100 make new
        # grid 100, if not check surrounding grids


    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))



    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        return path



    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
       #path is a list of grid Tuples
        #convert list of tuples to list of PoseStamped
        poseArray = [];
        #rospy.loginfo("NODES IN PATH: "+str(len(path)));
        for key in path:
            rospy.loginfo("key[0]: "+str(key[0])+", key[1]:"+str(key[1]));
            worldCoords = PathPlanner.grid_to_world(mapdata, key[0], key[1]);
            poseStamped = PoseStamped();
            poseStamped.pose.position.x = worldCoords.x;
            poseStamped.pose.position.y = worldCoords.y;
            header = Header()
            header.frame_id = "map"
            poseStamped.header = header;
            poseArray.append(poseStamped);

        pathHeader = Header();
        pathHeader.frame_id = "map";
        pathObject = Path();
        pathObject.header = pathHeader;
        pathObject.poses = poseArray;

        rospy.loginfo("Returning a Path message")
        return pathObject;



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
        """ #not needed anymore
        startPoint = Point();
        startPoint.x = start[0];
        startPoint.y = start[1];
        goalPoint = Point();
        goalPoint.x = goal[0];
        goalPoint.y = goal[1];
        """
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)



    def run(self):
        rospy.sleep(2.0)
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.calc_cspace(mapdata, 1)
        rospy.spin()



if __name__ == '__main__':
    PathPlanner().run()
