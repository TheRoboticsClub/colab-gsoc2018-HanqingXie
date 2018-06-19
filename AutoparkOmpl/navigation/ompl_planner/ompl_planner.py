#!/usr/bin/env python
from ompl import util as ou
from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og
from os.path import abspath, dirname, join
import sys
from functools import partial

import numpy as np
import math


class ompl_planner:
    def __init__(self, costMap, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, plannerType):
        self.costMap = costMap
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.plannerType = plannerType
        self.simpleSetup()

    def isStateValid(self, spaceInformation, state):
        # perform collision checking or check if other constraints are
        # satisfied
        wx = state.getX()
        wy = state.getY()
        
        if not self.costMap:
            tmp = True
            if wy < -4 and wy > 5:
                tmp = False
            elif wy < -0.5:
                if wx > 12 or wx < 5.5:
                    tmp = False 
        else:
            mPoint = self.costMap.worldToMap(wx, wy)
            cost = self.costMap.getCost(mPoint[0],mPoint[1])
            if cost >= 1:
                tmp = False
            else:
                tmp = True
        return spaceInformation.satisfiesBounds(state) and tmp
    
    def propagate(self, start, control, duration, state):
        state.setX( start.getX() + control[0] * duration * math.cos(start.getYaw()) )
        state.setY( start.getY() + control[0] * duration * math.sin(start.getYaw()) )
        state.setYaw(start.getYaw() + control[1] * duration)
    
    def simpleSetup(self):
        self.setSpace()
        self.setPose()
        self.setPlanner()

    def setSpace(self):
        # construct the state space we are planning in
        self.space = ob.SE2StateSpace()
        # set the bounds for the R^2 part of SE(2)
        self.bounds = ob.RealVectorBounds(2)
        if not self.costMap:
            self.bounds.setLow(-8)
            self.bounds.setHigh(20)
        else:
            ox = self.costMap.getOriginX()
            oy = self.costMap.getOriginY()
            size_x = self.costMap.getSizeInMetersX()
            size_y = self.costMap.getSizeInMetersY()
            low = min(ox, oy)
            high = max(ox+size_x, oy+size_y)
            print (low)
            print (high)
            self.bounds.setLow(low)
            self.bounds.setHigh(high)

        self.space.setBounds(self.bounds)

        # create a control space
        self.cspace = oc.RealVectorControlSpace(self.space, 2)
        # set the bounds for the control space
        cbounds = ob.RealVectorBounds(2)
        cbounds.setLow(0,-3)
        cbounds.setHigh(0,3)
        cbounds.setLow(1,-3)
        cbounds.setHigh(1,3)
        self.cspace.setBounds(cbounds)

        # define a simple setup class
        self.ss = oc.SimpleSetup(self.cspace)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(partial(self.isStateValid, self.ss.getSpaceInformation())))
        self.ss.setStatePropagator(oc.StatePropagatorFn(self.propagate))
    
    def setPose(self):
        # create a start state
        start = ob.State(self.space)
        start().setX(self.start_x)
        start().setY(self.start_y)
        start().setYaw(self.start_yaw)
        # create a goal state
        goal = ob.State(self.space)
        goal().setX(self.goal_x)
        goal().setY(self.goal_y)
        goal().setYaw(self.goal_yaw)

        # set the start and goal states
        self.ss.setStartAndGoalStates(start, goal, 0.05)
    
    def setPlanner(self):
        # (optionally) set planner
        self.si = self.ss.getSpaceInformation()

        if self.plannerType.lower() == "rrt":
            planner = oc.RRT(self.si)
        elif self.plannerType.lower() == "est":
            planner = oc.EST(self.si)
        elif self.plannerType.lower() == "kpiece1":
            planner = oc.KPIECE1(self.si)
        elif self.plannerType.lower() == "syclopest":
            # SyclopEST and SyclopRRT require a decomposition to guide the search
            decomp = MyDecomposition(32, self.bounds)
            planner = oc.SyclopEST(self.si, decomp)
        elif self.plannerType.lower() == "sycloprrt":
            # SyclopEST and SyclopRRT require a decomposition to guide the search
            decomp = MyDecomposition(32, self.bounds)
            planner = oc.SyclopRRT(self.si, decomp)
        else:
            OMPL_ERROR("Planner-type is not implemented in ompl control function.")

        self.ss.setPlanner(planner)
        # (optionally) set propagation step size
        self.si.setPropagationStepSize(.05)

    def solve(self, runtime=None):
        if not runtime:
            runtime = 10
        # attempt to solve the problem
        solved = self.ss.solve(runtime)

        if solved:
            # print the path to screen
            p = self.ss.getSolutionPath()
            #print("Found solution:\n%s" % ss.getSolutionPath().asGeometric().printAsMatrix())
            print("Found solution:\n%s" % self.ss.getSolutionPath().printAsMatrix())
            #p.interpolate()
            pathlist = [[] for i in range(3)]
            #print p.getStateCount()
            #print p.getControlCount() 
            for i in range(p.getControlCount()):
                x = p.getState(i+1).getX()
                y = p.getState(i+1).getY()
                yaw = p.getState(i+1).getYaw()
                pathlist[0].append(x)
                pathlist[1].append(y)
                pathlist[2].append(yaw)
                # print ('x = %f, y = %f, yaw = %f ' %(x,y,yaw))
                # t_x = p.getControl(i)[0]
                # t_y = p.getControl(i)[1]
                # t_yaw = p.getControl(i)[2]
                # print ('t_x = %f, t_y = %f, t_yaw = %f ' %(t_x,t_y,t_yaw)
            return pathlist
        else:
            print ("can't find path")
            return False
    
    def updateMap(self, costMap):
        self.costMap = costMap

    def updatePose(self,start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw):
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
    
    def updatePlannerType(self, plannerType):
        self.plannerType = plannerType

class MyDecomposition(oc.GridDecomposition):
    def __init__(self, length, bounds):
        super(MyDecomposition, self).__init__(length, 2, bounds)
    def project(self, s, coord):
        coord[0] = s.getX()
        coord[1] = s.getY()
    def sampleFullState(self, sampler, coord, s):
        sampler.sampleUniform(s)
        s.setXY(coord[0], coord[1])