import numpy as np
import time
import math
from scipy import interpolate  


class noHolomonicControl:
    def __init__(self):
        #self.pathlist = pathlist
        self.find_path = False
        self.PathId = 0
        self.getTarget = False
        self.v_old = 0
        self.w_old = 0
        self.v = 0
        self.w = 0
        self.e_old = [0,0,0]
        self.e = [0,0,0]

    def setPath(self, pathlist):
        self.pathlist = pathlist
    
    def getPath(self):
        return self.pathlist

    def findTarget(self,Pose):
        X = Pose[0]
        Y = Pose[1]
        Yaw = Pose[2]
        num = len(self.pathlist[0])-1
        TargetNowX = self.pathlist[0][self.PathId]
        TargetNowY = self.pathlist[1][self.PathId]
        dis = math.sqrt(pow(X-TargetNowX,2)+pow(Y-TargetNowY,2))
        if dis < 1:
            if num == self.PathId:
                if math.fabs(self.e[0]) < 0.1 and math.fabs(self.e[1])<0.5 and math.fabs(self.e[2])<0.1:
                    self.getTarget = True
            else:
                self.PathId += 1
        #elif(math.fabs(self.e[0]) < 0.01 and math.fabs(self.e[1])<1.5 and math.fabs(self.e[2])<0.01):
        #    self.PathId += 1

        print self.PathId
        target = [X,Y,Yaw]
        target[0] = self.pathlist[0][self.PathId]
        target[1] = self.pathlist[1][self.PathId]
        target[2] = self.pathlist[2][self.PathId]
        return target
    
    def isGetTarget(self):
        return self.getTarget

    def control(self, Pose):
        TargetPose = self.findTarget(Pose)
        print ("pose")
        print Pose
        print ("target")
        print TargetPose

        self.e[0] = math.cos(Pose[2])*(TargetPose[0] - Pose[0]) + math.sin(Pose[2])*(TargetPose[1] - Pose[1])
        self.e[1] = -math.sin(Pose[2])*(TargetPose[0] - Pose[0]) + math.cos(Pose[2])*(TargetPose[1] - Pose[1])
        self.e[2] = TargetPose[2] - Pose[2]
        print ("e:")
        print self.e
        print ("e_old:")
        print self.e_old
        self.v = self.v_old + 0.2*(self.e[0] - self.e_old[0]) + 0.01*self.e[0]
        self.w = self.w_old + 0.5*self.v*(self.e[1] - self.e_old[1])  + 0.001*self.e[1] + 0.2*(self.e[2] - self.e_old[2]) + 0.01*self.e[2]
        #self.pathlist
        print ("v w: ")
        print (self.v,self.w)
        print (self.v_old,self.w_old)
        if self.v > 0.5:
            self.w = self.w*0.5/self.v
            self.v = 0.5
        if self.v < -0.5:
            self.w = self.w*-0.5/self.v
            self.v = -0.5 
        self.v_old = self.v
        self.w_old = self.w
        self.e_old[0] = self.e[0]
        self.e_old[1] = self.e[1]
        self.e_old[2] = self.e[2]

        return [self.v, self.w]
        # self.motors.sendV(self.v)
        # self.motors.sendW(self.w)
