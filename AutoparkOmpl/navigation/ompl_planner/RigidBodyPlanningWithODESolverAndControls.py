#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll

from math import sin, cos, tan
from functools import partial

from ompl import util as ou
from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og

from os.path import abspath, dirname, join
import numpy as np
import math 

import matplotlib.pyplot as plt
## @cond IGNORE
# a decomposition is only needed for SyclopRRT and SyclopEST
class RigidBodyPlanningWithODESolverAndControls:
    def __init__(self):
        self.a = 0

    def kinematicCarODE(self, q, u, qdot):
        theta = q[2]
        carLength = 5
        qdot[0] = u[0] * cos(theta)
        qdot[1] = u[0] * sin(theta)
        qdot[2] = u[0] * tan(u[1]) #/ carLength

    def isStateValid(self, spaceInformation, state):
        # perform collision checking or check if other constraints are
        # satisfied
        x = state.getX()
        y = state.getY()
        # r = math.sqrt((x-0)*(x-0) + (y-0)*(y-0)) - 4
        #print state
        tmp = True
        if y < -4 and y > 5:
           tmp = False
        elif y < -0.5:
            if x > 12 or x < 5.5:
                tmp = False 
        
        return spaceInformation.satisfiesBounds(state) and tmp


    def plan(self, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, ):
        # construct the state space we are planning in
        self.space = ob.SE2StateSpace()

        # set the bounds for the R^2 part of SE(2)
        bounds = ob.RealVectorBounds(2)
        low = min(start_x, start_y, goal_x, goal_y)
        high = max(start_x, start_y, goal_x, goal_y)
        print (low)
        print (high)
        bounds.setLow(-8)
        bounds.setHigh(20)
        self.space.setBounds(bounds)

        # create a control space
        self.cspace = oc.RealVectorControlSpace(self.space, 2)

        # set the bounds for the control space
        cbounds = ob.RealVectorBounds(2)
        cbounds.setLow(0,-1.5)
        cbounds.setHigh(0,1.5)
        cbounds.setLow(1,-2.5)
        cbounds.setHigh(1,2.5)
        self.cspace.setBounds(cbounds)

        # define a simple setup class
        self.ss = oc.SimpleSetup(self.cspace)
        validityChecker = ob.StateValidityCheckerFn(partial(self.isStateValid, self.ss.getSpaceInformation()))
        self.ss.setStateValidityChecker(validityChecker)     
        ode = oc.ODE(self.kinematicCarODE)
        odeSolver = oc.ODEBasicSolver(self.ss.getSpaceInformation(), ode)
        propagator = oc.ODESolver.getStatePropagator(odeSolver)
        self.ss.setStatePropagator(propagator)

        # create a start state
        start = ob.State(self.space)
        start().setX(start_x)
        start().setY(start_y)
        start().setYaw(start_yaw)

        # create a goal state
        goal = ob.State(self.space)
        goal().setX(goal_x)
        goal().setY(goal_y)
        goal().setYaw(goal_yaw)

        # set the start and goal states
        self.ss.setStartAndGoalStates(start, goal, 0.05)

        #(optionally) set planner
        self.si = self.ss.getSpaceInformation()
        #planner = oc.RRT(self.si)
        #planner = oc.EST(self.si)
        planner = oc.KPIECE1(self.si) # this is the default
        # SyclopEST and SyclopRRT require a decomposition to guide the search
        #decomp = MyDecomposition(32, bounds)
        #planner = oc.SyclopEST(self.si, decomp)
        #planner = oc.SyclopRRT(self.si, decomp)
        self.ss.setPlanner(planner)
        # (optionally) set propagation step size
        self.si.setPropagationStepSize(.05)

        # attempt to solve the problem
        solved = self.ss.solve(100.0)

        if solved:
            # print the path to screen
            p = self.ss.getSolutionPath()
            print("Found solution:\n%s" % self.ss.getSolutionPath().asGeometric().printAsMatrix())
            
            pathlist = [[] for i in range(3)]
            #print p.getControlCount() 
            for i in range(p.getControlCount()):
                x = p.getState(i+1).getX()
                y = p.getState(i+1).getY()
                yaw = p.getState(i+1).getYaw()
                pathlist[0].append(x)
                pathlist[1].append(y)
                pathlist[2].append(yaw)
            return pathlist
        else:
            print ("can't find path")
            return 0

class MyDecomposition(oc.GridDecomposition):
    def __init__(self, length, bounds):
        super(MyDecomposition, self).__init__(length, 2, bounds)
    def project(self, s, coord):
        coord[0] = s.getX()
        coord[1] = s.getY()
    def sampleFullState(self, sampler, coord, s):
        sampler.sampleUniform(s)
        s.setXY(coord[0], coord[1])
## @endcond

if __name__ == "__main__":
    
    planner = RigidBodyPlanningWithODESolverAndControls()
    pathlist = planner.plan(6,3,0,7.25,-3,0)
    plt.figure(1)
    plt.plot(pathlist[0], pathlist[1])
    num = len(pathlist[0])
    for i in range(num):
        x1 = pathlist[0][i]
        y1 = pathlist[1][i]
        yaw = pathlist[2][i]
        x2 = x1 + 0.5*cos(yaw)
        y2 = y1 + 0.5*sin(yaw)
        plt.plot([x1,x2],[y1,y2])
    plt.show()
