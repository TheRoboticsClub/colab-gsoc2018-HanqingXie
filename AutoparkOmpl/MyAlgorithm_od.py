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
            
    def findTarget(self,X,Y,Yaw):
        num = len(self.pathlist[0])-1
        TargetNowX = self.pathlist[0][self.PathId]
        TargetNowY = self.pathlist[1][self.PathId]
        dis = math.sqrt(pow(X-TargetNowX,2)+pow(Y-TargetNowY,2))
        if dis < 0.1:
            if num == self.PathId:
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
        if self.find_path:
            if self.getTarget:
                print "get Target"
                self.motors.sendV(0)
                self.motors.sendW(0)
                return
            X = self.pose3d.getPose3d().x
            Y = self.pose3d.getPose3d().y
            Yaw = self.pose3d.getPose3d().yaw
            TargetPose = self.findTarget(X,Y,Yaw)
            print ("pose")
            print (X,Y,Yaw)
            print ("target")
            print TargetPose
            yaw_target = math.atan2(TargetPose[1]-Y,TargetPose[0]-X)
            
            yaw_dis = TargetPose[2] - Yaw
            yaw_target_dis = yaw_target - Yaw
            dis = math.sqrt(pow(X-TargetPose[0],2)+pow(Y-TargetPose[1],2))
            print ("yaw_target = %f"%yaw_target)
            print ("yaw_dis = %f, yaw_target_dis = %f"%(yaw_dis,yaw_target_dis))

            if yaw_dis > 3.14:
                yaw_dis -= 6.28
            if yaw_dis < -3.14:
                yaw_dis += 6.28 
            if yaw_target_dis > 3.14:
                yaw_target_dis -= 6.28
            if yaw_target_dis < -3.14:
                yaw_target_dis += 6.28 

            if min(abs(yaw_dis),abs(yaw_target_dis)) > 1 or min(3.14-abs(yaw_target_dis),abs(yaw_target_dis)) > abs(yaw_dis)+0.1:
                v = 0.05
                if yaw_target_dis > 1.57:
                    w = -3.14 + yaw_target_dis
                elif yaw_target_dis < -1.57:
                    w = 3.14 + yaw_target_dis
                else:
                    w = yaw_target_dis
                
            else:
                if yaw_target_dis > 1.57 or yaw_target_dis < -1.57:
                    v = -0.2
                else:
                    v = 0.2
                if yaw_dis != 0:
                    r = dis/yaw_dis
                    w = v/r
                else:
                    w = 0
            #self.pathlist
            print (v,w)
            self.motors.sendV(v)
            self.motors.sendW(w)
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

        
