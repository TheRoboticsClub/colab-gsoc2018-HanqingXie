import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
import cv2
from math import pi as pi

from navigation.ompl_planner.ompl_planner import ompl_planner
from navigation.control.control import noHolomonicControl
from navigation.path_smooth.smoothPath import smooth
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


        self.control = noHolomonicControl()
        self.smooth = smooth()
        self

        self.find_path = False
        # self.PathId = 0
        self.getTarget = False
        # self.v_old = 0
        # self.w_old = 0
        # self.v = 0
        # self.w = 0
        # self.e_old = [0,0,0]
        # self.e = [0,0,0]

        

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

            data = self.control.control(Pose)
            self.motors.sendV(data[0])
            self.motors.sendW(data[1])
            self.getTarget = self.control.isGetTarget()

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
            ompl_sol = ompl_planner(None, startX, startY, startYaw, goalX, goalY, goalYaw, "rrt", True, True)
            self.pathlist = ompl_sol.omplRunOnce()
            #print self.pathlist

            if self.pathlist[0][0] > startX:
                self.pathlist[0][0] = startX - 0.1
            

            self.smooth.setPath(self.pathlist)
            
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

            self.pathlist = self.smooth.Floyd(self.pathlist)
            self.pathlist = self.smooth.checkYaw(self.pathlist, [startX,startY,startYaw])
            num = len(self.pathlist[0])

                
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

            # self.smooth.inter(self.pathlist)
            plt.show()
            
            self.control.setPath(self.pathlist)
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

    
        
#region floyd
