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
from navigation.path_processing.smoothPath import smooth
from navigation.map.occGridMap import occGridMap
from navigation.navigation import navigation
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

        self.navigation = navigation('/home/hywel/JdeRobot_ws/colab-gsoc2018-HanqingXie/AutoparkOmpl/init_map.ppm',-250, -250, 1.25)

        self.find_path = False

        self.buildMap = occGridMap()

        

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
            pose = [0,0,0]
            pose[0] = self.pose3d.getPose3d().x
            pose[1] = self.pose3d.getPose3d().y
            pose[2] = self.pose3d.getPose3d().yaw

            data = self.navigation.path_following(pose)
            self.motors.sendV(data[0])
            self.motors.sendW(data[1])

        else:
            self.motors.sendV(0)
            self.motors.sendW(0)
            #car (6,3)
            #tartget (7.25,-3)
            pose = [0, 0, 0]
            pose[0] = self.pose3d.getPose3d().x
            pose[1] = self.pose3d.getPose3d().y
            pose[2] = self.pose3d.getPose3d().yaw
            goal_pose = [0,0,0]
            goal_pose[0] = 96
            goal_pose[1] = 118
            goal_pose[2] = 0

            self.find_path = self.navigation.path_planning(pose,goal_pose)

        '''
        pose = [0,0,0]
        pose[0] = self.pose3d.getPose3d().x
        pose[1] = self.pose3d.getPose3d().y
        pose[2] = self.pose3d.getPose3d().yaw

        laser0_data = self.laser1.getLaserData()
        laser0 = self.parse_laser_data(laser0_data)
        self.navigation.updateMap(laser0,0,pose)

        laser1_data = self.laser2.getLaserData()
        laser1 = self.parse_laser_data(laser1_data)
        self.navigation.updateMap(laser1,1,pose)

        laser2_data = self.laser3.getLaserData()
        laser2 = self.parse_laser_data(laser2_data)
        self.navigation.updateMap(laser2,2,pose)
        '''
    
    def parse_laser_data(self, laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = math.radians(i)
            laser.append([dist])#[(dist, angle)]
        return laser

#laser1~ 0 front      
# laser2 ~ 2 
# laser3 ~ 1
#region floyd
