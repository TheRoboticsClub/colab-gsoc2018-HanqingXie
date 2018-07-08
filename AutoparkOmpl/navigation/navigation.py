
from ompl_planner.ompl_planner import ompl_planner
from control.control import noHolomonicControl
from path_processing.smoothPath import smooth
from map.occGridMap import occGridMap

import matplotlib.pyplot as plt
import math

class navigation:
    def __init__(self):
        self.control = noHolomonicControl()
        self.smooth = smooth()
        self.occGridMap = occGridMap()
        self.getTarget = False
    
    def path_planning(self,current_pose,goal_pose,costMap):

        startX = current_pose[0]
        startY = current_pose[1]
        startYaw = current_pose[2]
        goalX = goal_pose[0]
        goalY = goal_pose[1]
        goalYaw = goal_pose[2]
        print (startX, startY, startYaw, goalX, goalY, goalYaw)

        ompl_sol = ompl_planner(None, startX, startY, startYaw, goalX, goalY, goalYaw, "rrt", True, True)
        path_list = ompl_sol.omplRunOnce()
        #print self.pathlist
        if path_list:
            path_list = self.path_smooth(path_list,current_pose)

            self.control.setPath(path_list)
            return True
        else:
            return False
    
    def path_smooth(self,path_list,current_pose):
        plt.figure(1)
        plt.plot(path_list[0], path_list[1])
        num = len(path_list[0])
        for i in range(num):
            x1 = path_list[0][i]
            y1 = path_list[1][i]
            yaw = path_list[2][i]
            x2 = x1 + math.cos(yaw)
            y2 = y1 + math.sin(yaw)
            plt.plot([x1,x2],[y1,y2])
        plt.savefig("figure1.jpg") 
        self.smooth.setPath(path_list)
        path_list = self.smooth.Floyd(path_list)
        path_list = self.smooth.checkYaw(path_list, current_pose)

        plt.figure(2)
        plt.plot(path_list[0], path_list[1])
        for i in range(num):
            x1 = path_list[0][i]
            y1 = path_list[1][i]
            yaw = path_list[2][i]
            x2 = x1 + math.cos(yaw)
            y2 = y1 + math.sin(yaw)
            plt.plot([x1,x2],[y1,y2])
        plt.savefig("figure2.jpg")

        return path_list

    def path_following(self, current_pose):
        if self.getTarget:
            print "get Target"
            print self.control.getPath()
            return [0,0]
            
        Pose = [0,0,0]
        Pose[0] = current_pose[0]
        Pose[1] = current_pose[1]
        Pose[2] = current_pose[2]

        data = self.control.control(Pose,2)
        self.getTarget = self.control.isGetTarget()
        return data

    
    def path_check(self,path_smooth, costmap):
        a = 1
    

    def updateMap(self,laser, id, pose):
        self.occGridMap.registerScan(pose,id,laser)
        self.occGridMap.drawMap()