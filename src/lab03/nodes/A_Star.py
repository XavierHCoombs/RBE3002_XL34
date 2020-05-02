#!/usr/bin/env python
import rospy
import math
import sys
import priority_queue
import map_helper
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PoseStamped,Pose, Quaternion, Point, PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.srv import GetPlan
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from std_msgs.msg import Header
from beginner_tutorials.srv import *

class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """

        rospy.init_node("a_star")  # start node
        
        #Properties of the Enviroment
        self.frontier = priority_queue.PrioriyQueue()
        self.explored = []
        self.unexplored = []
        self.front = []
        self.map = OccupancyGrid()
        
        #Publishers
        self.exPub =  rospy.Publisher('/explored', GridCells, queue_size=1)
        self.unexPub = rospy.Publisher('/unexplored', GridCells, queue_size=1)
        self.frontPub = rospy.Publisher('/frontier', GridCells, queue_size=1)
        self.pathPub = rospy.Publisher('/path', GridCells, queue_size=1)
        
        #Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.set_map)
        
        #wait to recieve the map
        rospy.sleep(5)
        #services
        s=rospy.Service('a_star', GetPlan, self.handle_a_star)
 

    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        print("Returning path...")
        return  self.a_star(req.start, req.goal)
       #pass

       

    def dynamic_map_client(self):

        """
            Service call to get map and set class variables
            This can be changed to call the expanded map
            :return:
        """
        #get resolution of the map and round the incoming values
        res=round(self.map.info.resolution,1)
        start_x=round(start[0],1)
        start_y=round(start[1],1)
        goal_x=round(goal[0],1)
        goal_y=round(goal[1],1)
        
        start = (start_x,start_y)
        goal = (goal_x, goal_y)
        
        return (start, goal)
        
        #pass


    def a_star(self, start, goal):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
        #Initializinf A*
        self.frontier.put(start, 0)
        prev_cell = {}
        curr_path = {}
        prev_cell[start] = None
        curr_path[start] = 0
        
        finalPath = {}
        
        while not self.frontier.empty():
            curr_cell=self.frontier.get()
            self.explored.append(curr_cell)
            
            #If the current cell is the goal, 1.)add it to the finalPath, 2.) add it to the explored list of cells
            if curr_cell == goal:
                    print ("reached goal")
                    finalPath[curr_cell]=(curr_path[prev_cellA], prev_cellA)
                    self.explored.append(curr_cell)
                    self.explored.appemd(goal)
                    self.paint_cells(self.fron, self.explored)
                    rospy.sleep(0.2)
                    break
             
            #Keep track of prev_cell inorder to reconstruct the finalPath
            prev_cell= curr_cell
            
            for next in map_helper.get_neighbors(curr_cell, self.map):
                next_path= round(curr_path[curr_cell]+ self.move_cost (curr_cell, next),2)
                if next not in curr_path or next_path<<curr_path[next]:
                    curr_path[next]= next_path
                    priority= next_path+self.euclidean_heuristic(next,goal)
                    self.frontier.put(next,priority)
                    self.front.append(next)
                    
                    #If no Prev_cell set it to origin
                    if prev_cell.get(curr_cell) is None:
                        self.explored.append((0,0,0))
                    else:
                        self.explored.append(prev_cell.get(curr_cell))
                    prev_cell[next]= curr_cell
                    
            #Paint the explored cells and frontier
            self.paint_cells(self.front, self.explored)
        print("A-Star Completed")
        return finalPath
    
        #pass

      
    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        """           #Can delete now 
        x1 = point1[0]
        y1 = point1[1]

        x2 = point2[0]
        y2 = point2[1]

        out =  math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return out
        """
        dX = abs(point2[0]-point1[0])
        dY = abs(point2[1-point1[1])
        out = math.sqrt((dX**2) + (dY**2))
        return round(out, 1)
                        
   #pass

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        changeX = abs(next[0]- current[0])
        changeY = abs(next[1]- current[1])
        out = round(changeX +changeY)
        return out      
                        
       #pass


    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
       """
       outputList=list()
       outputList.append(goal)

       prevCellA = came_from.get(goal)
       prevCell=prevCellA[1]

       while prevCell != start:
           outputList.append(prevCell)

           cell=came_from.get(prevCell)
           outputList.append(cell[1])
           prevCell = cell[1]

       outputList.append(start)
                        
       return outputList             
      #pass
  

    def optimize_path(self, path):
        """
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        optPath=list()
        for cell in path:
            if cell not in optPath:
                optPath.append(cell)
        return optPath
                        
      # pass

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        #If cell is in both explored and frontier, remove it from frontier
        for cell in frontier:
            if cell in explored:
                frontier.remove(cell)
        front=map_helper.to_cells(frontier,self.map)
        self.frontPub.publish(front)
        explore=map_helper.to_cells(explored,self.map)
        self.exPub.publish(explore)
        pass


    def publish_path(self, points):
        """
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path
            :return: Path()
        """
        path=[]
        i=0
        poseArray=[]
        #invert the list so the points are in order from the start to goal
        points=list(reversed(points))
        # print(points)

        while i<len(points):
            k=points[i]
            point=Point()
            point.x=k[0]
            point.y=k[1]
            point.z=k[2]

            orientation=Quaternion()

            #####Calculate the appropriate orientation for the robot to rotate to to move to each cell in a straight line
            if i==0:

                orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,0)
            else:
                #in the global frame
                ##X CHANGES
                #was previously going directly to right, goal is further the right. continue straight
                if prevPt.x < point.x and prev_yaw==0:
                    print(prevPt.x)
                    print(point.x)
                    print("continue to the right")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,0)

                #was previously going to the left, goal goal is further to the left, continue straight
                elif prevPt.x > point.x and prev_yaw==math.pi:
                    print("continue to the left")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,math.pi)

                #was previously going to the right, goal goal is  to the left, turn around
                elif prevPt.x > point.x and prev_yaw==0:
                    print("turn around")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,math.pi)

                #was prevously going up (posy) and goal is to the right, then turn to the right
                elif prevPt.x < point.x and prev_yaw==math.pi/2:
                    print("gone up now turn right")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,0)

                #was previsoulsy going up (posy) and goal is to the left, then turn to the left
                elif prevPt.x>point.x and prev_yaw==math.pi/2:
                    print("gone up now turn left")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,math.pi)

                #was prevously going down (negy) and goal is to the left(world), then turn to the left(world)
                elif prevPt.x > point.x and prev_yaw==(-math.pi/2):
                    print("gone down now turn left")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,math.pi)

                #was previsoulsy going down (negy) and goal is to the right (world), then turn to the right
                elif prevPt.x>point.x and prev_yaw==(3*math.pi/2):
                    print("gone down now turn right")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,2*math.pi)


                #YCHANGES
                #WAS previsoulsy going to the right, goal is up(posy) turn to the left
                elif prevPt.y < point.y and prev_yaw==0:
                    print("going right, turn up")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,math.pi/2)

                #was previsoulsy going right(world), goal is down so turn right, (down)
                elif prevPt.y>point.y and prev_yaw == 0:
                    print("going right, turn down")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,3*math.pi/2)

                #was previsoulsy going up, goal is still up so continue straight
                elif prevPt.y < point.y and prev_yaw == math.pi/2:
                    print("going up, continue up")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,math.pi/2)

                #was previsoulsy going left(world), goal is down so turn left, (down)
                elif prevPt.y>point.y and prev_yaw == math.pi:
                    print("going left, turn down")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,-math.pi/2)

                #was previsoulsy going left(world), goal is up so turn right, (up)
                elif prevPt.y<point.y and prev_yaw == math.pi:
                    print("going left, turn up")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,math.pi/2)

                #was previsoulsy going down(world), goal is down so turn right, (down)
                elif prevPt.y>point.y and prev_yaw == (3*math.pi/2):
                    print("going down, continue down")
                    orientation.x,orientation.y,orientation.z,orientation.w=quaternion_from_euler(0,0,3*math.pi/2)


            #keep track of the previous point
            prevPt=point
            #create header for the stamped poses
            header2=Header()
            header2.frame_id="map"
            #convert points to poses
            pose=Pose()
            pose.position=point
            pose.orientation=orientation

            poseStamped=PoseStamped()
            poseStamped.pose=pose
            poseStamped.header=header2
            #pose array created for the path
            poseArray.append(poseStamped)

            #keep track of the previous pose orientation for the calcualtions above
            prevOrient=pose.orientation
            q = [prevOrient.x, prevOrient.y, prevOrient.z, prevOrient.w]
            prev_roll, prev_pitch, prev_yaw = euler_from_quaternion(q)

            i+=1

        #creating path object to return
        path=Path()
        header1=Header()
        header1.frame_id="map"
        path.poses=poseArray
        path.header=header1

        #publish GridCells
        pointArray=[]
        for k in path.poses:
            pointArray.append(k.pose.position)

        gridCell=map_helper.to_cells(pointArray,self.map)
        self.pubPath.publish(gridCell)
        print("pubishing GRIDCELLS")
        return path

        #pass


if __name__ == '__main__':
    a_star=A_Star()
    while not rospy.is_shutdown():
        rospy.spin()
    pass
