from sensors import sensor
import numpy as np
import cv2

import threading
import time
from datetime import datetime
from ompls.OptimalPlanning import optimalPlanning
from ompls.Point2DPlanning import Plane2DEnvironment

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        sensor.getPathSig.connect(self.generatePath)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


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

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button. 
        Call to grid.setPath(path) mathod for setting the path. """
    def generatePath(self, list):
        print("LOOKING FOR SHORTER PATH")
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        planner = str(list[0])
        objective = str(list[1])
        runtime = str(list[2])
        
        #print mapIm

        print planner
        print objective
        print runtime


        # optimal_planning = optimalPlanning()
        # optimal_planning.setStart(gridPos[0],gridPos[1])
        # optimal_planning.setGoal(dest[0],dest[1])
        # optimal_planning.SetMap(mapIm)
        # optimal_planning.plan(float(runtime), planner, objective, None)
        fname = '/home/hywel/JdeRobot_ws/Academy/src/navigation_ompl/resources/images/cityLargeBin.ppm'
        env = Plane2DEnvironment(fname)

        if env.plan(gridPos[0],gridPos[1], dest[0],dest[1]):
            env.recordSolution()
            env.save("result_demo.ppm")
            pathlist = env.getPath()
        
        patharray = np.array(pathlist)
        patharray = np.rint(patharray)
        #print patharray
        size = patharray.shape
        #print size
        num = 0
        pathX_old = -1
        pathY_old = -1
        for i in range(size[1]):
            pathX = patharray[0][i]
            pathY = patharray[1][i]
            if pathX != pathX_old or pathY != pathY_old:
                self.grid.setPathVal (int(pathX), int(pathY), num)
                num += 1
                pathX_old = pathX
                pathY_old = pathY
                print pathX
                print pathY

        self.grid.setPathFinded()

        #Represent the Gradient Field in a window using cv2.imshow

    """ Write in this mehtod the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """

    def execute(self):
        # Add your code here
        print("GOING TO DESTINATION")

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.vel.setV(10)
        #self.vel.setW(5)
        #self.vel.sendVelocities()
