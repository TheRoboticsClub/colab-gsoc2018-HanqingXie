import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
import cv2
from math import pi as pi
from ompl_solution.RigidBodyPlanningWithControls import RigidBodyPlanning
import matplotlib.pyplot as plt
from scipy import interpolate  

time_cycle = 80


class MyAlgorithm(threading.Thread):

    def __init__(self, pose3d, laser1, laser2, laser3, motors):
        self.pose3d = pose3d
        self.laser1 = laser1
        self.laser2 = laser2
        self.laser3 = laser3
        self.motors = motors

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

        self.find_path = False
        self.PathId = 0
        self.getTarget = False
        self.v_old = 0
        self.w_old = 0
        self.v = 0
        self.w = 0
        self.e_old = [0,0,0]
        self.e = [0,0,0]

        self.last_trax = [] 

    def run (self):

        while (not self.kill_event.is_set()):
           
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)


    def stop (self):
        self.stop_event.set()


    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()


    def kill (self):
        self.kill_event.set()
            
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

    def execute(self):
        #print "Runing"
        #self.find_path = True
        #self.pathlist = [[13.5,12,10.5,9,7.5],[2.5,1.5,0,-1.5,-2.5],[0,0.5,1,0.5,0]]
        if self.find_path:
            if self.getTarget:
                print "get Target"
                self.motors.sendV(0)
                self.motors.sendW(0)
                print self.pathlist
                return
            Pose = [0,0,0]
            Pose[0] = self.pose3d.getPose3d().x
            Pose[1] = self.pose3d.getPose3d().y
            Pose[2] = self.pose3d.getPose3d().yaw
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

            self.motors.sendV(self.v)
            self.motors.sendW(self.w)

        else:
            #car (6,3)
            #tartget (7.25,-3)
            startX = self.pose3d.getPose3d().x
            startY = self.pose3d.getPose3d().y
            startYaw = self.pose3d.getPose3d().yaw
            goalX = 7.25
            goalY = -3
            goalYaw = 0
            print (startX, startY, startYaw, goalX, goalY, goalYaw)
            ompl_sol = RigidBodyPlanning()
            self.pathlist = ompl_sol.plan(startX, startY, startYaw, goalX, goalY, goalYaw)
            #print self.pathlist

            if self.pathlist[0][0] > startX:
                self.pathlist[0][0] = startX - 0.1
            
            plt.figure(1)
            plt.plot(self.pathlist[0], self.pathlist[1])
            num = len(self.pathlist[0])
            for i in range(num):
                x1 = self.pathlist[0][i]
                y1 = self.pathlist[1][i]
                yaw = self.pathlist[2][i]
                x2 = x1 + math.cos(yaw)
                y2 = y1 + math.sin(yaw)
                plt.plot([x1,x2],[y1,y2])
            plt.savefig("figure1.jpg") 
            # x = y = np.arange(-5, 5, 0.1)
            # x, y = np.meshgrid(x,y)
            # plt.contour(x, y, x**2 + y**2, [16])
            self.Floyd()
            num = len(self.pathlist[0])
            for i in range(num-1):
                if (i == 0):
                    x0 = startX
                    y0 = startY
                else:
                    x0 = self.pathlist[0][i-1]
                    y0 = self.pathlist[1][i-1]
                x1 = self.pathlist[0][i]
                y1 = self.pathlist[1][i]
                x2 = self.pathlist[0][i+1]
                y2 = self.pathlist[1][i+1]
                vector1 = [x1-x0,y1-y0]
                vector2 = [x2-x1,y2-y1]
                yaw1 = math.atan2(vector1[1],vector1[0])
                yaw2 = math.atan2(vector2[1],vector2[0])
                print("yaw1")
                print yaw1
                print("yaw2")
                print yaw2
                yawtmp = (yaw2+yaw1)/2
                if (i==0):
                    yawtmp = self.setYaw(yawtmp,startYaw)
                else:
                    yawtmp = self.setYaw(yawtmp,self.pathlist[2][i-1])
                print "yawtmp"
                print yawtmp
                self.pathlist[2][i] = yawtmp
                
            plt.figure(2)
            plt.plot(self.pathlist[0], self.pathlist[1])
            
            for i in range(num):
                x1 = self.pathlist[0][i]
                y1 = self.pathlist[1][i]
                yaw = self.pathlist[2][i]
                x2 = x1 + math.cos(yaw)
                y2 = y1 + math.sin(yaw)
                plt.plot([x1,x2],[y1,y2])
            plt.savefig("figure2.jpg")
            self.inter()
            plt.show()
            self.find_path = True

            
        
        
        #laser_data = self.laser2.getLaserData()
        #laser = self.parse_laser_data(laser_data)
        #print laser
        
        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.motors.sendV(10)
        #self.motors.sendW(5)
        
        # TODO
    
    def parse_laser_data(self, laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = math.radians(i)
            laser += [(dist, angle)]
        return laser

    def setYaw(self, newYaw, lastYaw):
        dis = newYaw - lastYaw
        if math.fabs(dis) > 1.57:
            if newYaw >= 0:
                newYaw = -(3.14 - newYaw)
            else:
                newYaw = 3.147 + newYaw 
        return newYaw
        
#region floyd
    def Floyd(self):
        numLen = len(self.pathlist[0])
        if (numLen > 2):
            
            vector = [self.pathlist[0][numLen -1] - self.pathlist[0][numLen - 2], self.pathlist[1][numLen -1] - self.pathlist[1][numLen - 2],self.pathlist[2][numLen -1] - self.pathlist[2][numLen - 2]]
            tempvector = [0,0,0]
            #for (int i = numLen - 3; i>= 0; i--)
            i = numLen - 3
            while (i >= 0):
                tempvector = [self.pathlist[0][i+1] - self.pathlist[0][i], self.pathlist[1][i+1] - self.pathlist[1][i],self.pathlist[2][i+1] - self.pathlist[2][i]]
                if math.fabs(math.atan2(vector[1],vector[0])-math.atan2(tempvector[1],tempvector[0])) < 0.001:
                    del self.pathlist[0][i+1]
                    del self.pathlist[1][i+1]
                    del self.pathlist[2][i+1]
                else:
                    vector = tempvector
                i -= 1
        numLen = len(self.pathlist[0])
        #for (int i = numLen-1; i >= 0; i--) 
        i = numLen-1
        while(i>=0):
            j = 0
            while (j<=i-1):
            #for (int j = 0; j<= i-1; j++)
            
                if (self.CheckCrossNoteWalkable(i,j)):
                    k = i-1
                    while(k>=j):
                    #for (int k = i-1; k>=j; k--):
                        del self.pathlist[0][k]
                        del self.pathlist[1][k]
                        del self.pathlist[2][k]
                        k -= 1
                    i=j
                    break
                j += 1
            i -= 1

        print self.pathlist
    
    def CheckCrossNoteWalkable(self, i ,j):
        x1 = self.pathlist[0][i]
        y1 = self.pathlist[1][i]
        x2 = self.pathlist[0][j]
        y2 = self.pathlist[1][j]
        dis = math.sqrt(pow(x1-x2,2)+pow(y1-y2,2))
        return dis < 1

    def inter(self):
        x = self.pathlist[0]
        y = self.pathlist[1]
        xnew=np.linspace(min(x),max(x),201)
        plt.figure(3)
        plt.plot(x,y,"ro")  
  
        for kind in ["nearest","zero","slinear","quadratic","cubic"]:
            #"nearest","zero"
            #slinear
            #"quadratic","cubic"
            f=interpolate.interp1d(x,y,kind=kind)  
            ynew=f(xnew)
            plt.plot(xnew,ynew,label=str(kind))
            plt.savefig("figure3.jpg")  


    def sub_spl(self,x,y):  
        if len(x) == 2:  
            return (y[1] -y[0])/(x[1] - x[0])  
        return (self.sub_spl(x[1:],y[1:]) - self.sub_spl(x[:len(x)-1],y[:len(y) - 1]))/(x[len(x) - 1] - x[0])  
    def three_spline_trax(self,tup_x,tup_y,s0,sn):  
        trax = []  
        h = []  
        for i in range(len(tup_x) - 1):  
            h.append(tup_x[i+1] - tup_x[i])  
        d = [0 for i in  range(len(tup_x))]  
        d[0] = 6/h[0] * (self.sub_spl(tup_x[:2],tup_y[:2]) - s0)  
        d [-1] = 6/h[-1] * (sn - self.sub_spl(tup_x[-2:],tup_y[-2:]))  
        u = [0 for i in  range(len(tup_x)-1)]  
        for j in  range(1,len(tup_x) - 1):  
            d[j] = 6 * self.sub_spl(tup_x[j-1:j+2],tup_y[j-1:j+2])  
            u[j] = h[j-1]/(h[j-1] + h[j])  
        l =  [1-i for i in u]  
        u.append(1) #un = 1  
        for i in range(len(tup_x)):  
            trax.append([0 for j in range(len(tup_x))])  
            trax[i][i] = 2  
    
        for i in range(len(tup_x) - 1):  
            trax[i][i+1] = l[i]  
            trax[i+1][i] = u[i+1]  
            trax[i].append(d[i])  
        trax[-1].append(d[-1])  
        return trax,h  


    def return_trax(self,trax,m,n,num):  
        if num is 1:  
            self.last_trax.append(trax[n-1][n-1:])  
            return   
        else:  
            Max = abs(trax[0][0])  
            t1 = 0  
            for i in  range(1,m):  
                if abs(trax[i][0]) > Max:  
                    Max = trax[i][0]  
                    t1 = i  
            trax[0],trax[t1] = trax[t1],trax[0]  
            tmp2 = []  
            self.last_trax.append(trax[0])  
            for i in range(1,n):  
                tmp = -trax[i][0]/trax[0][0]  
                def cal(tup):  
                    return tup[0] + tup[1] * tmp  
                tmp2.append(map(cal,zip(trax[i][1:],trax[0][1:])))  
            trax = tmp2  
            return self.return_trax(trax,m-1,n-1,num-1)  
    
    def s_um(self,tup):  
        return reduce(lambda x,y:x*y,tup)  
    

    def calculate(self,trax,m):   
        l = [0 for i in range(m)]  
        for i in range(m-1,-1,-1):  
            if i == m-1:  
                l[i] = trax[i][m-i]/trax[i][0]  
            else:  
                tmp3 = trax[i][m-i]  
                tmp3 -= sum(map(self.s_um,zip(l[i:],trax[i][:m-i])))  
                l[i] = tmp3/trax[i][0]  
        return l  
    
    def res(self,l,tup_x1,tup_y1,h):  
        def outcome(x):  
            for j in  range(len(tup_x1) - 1):  
                if tup_x1[j] <= x <= tup_x1[j+1]:  
                    sum_func = l[j]*(tup_x1[j+1] - x)**3/(6*h[j]) + l[j+1]*(x - tup_x1[j])**3/(6*h[j])+(tup_y1[j] - l[j]/6*h[j]**2)*(tup_x1[j+1] - x)/h[j] + (tup_y1[j+1]-l[j+1]/6*h[j]**2)*(x - tup_x1[j])/h[j]  
                    return sum_func  
        return outcome  