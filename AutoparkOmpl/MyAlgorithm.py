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
                if dis < 0.2 and abs(Yaw - self.pathlist[2][self.PathId]) < 0.2:
                    self.getTarget = True
            else:
                self.PathId += 1
            
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
            self.w = self.w_old + 0.5*self.v*(self.e[1] - self.e_old[1]) + 0.2*(self.e[2] - self.e_old[2]) + 0.01*self.e[2]
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
            goalYaw = startYaw
            print (startX, startY, startYaw, goalX, goalY, goalYaw)
            ompl_sol = RigidBodyPlanning()
            self.pathlist = ompl_sol.plan(startX, startY, startYaw, goalX, goalY, goalYaw)
            #print self.pathlist

            plt.figure(1)
            plt.plot(self.pathlist[0], self.pathlist[1])
            # x = y = np.arange(-5, 5, 0.1)
            # x, y = np.meshgrid(x,y)
            # plt.contour(x, y, x**2 + y**2, [16])
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

        
