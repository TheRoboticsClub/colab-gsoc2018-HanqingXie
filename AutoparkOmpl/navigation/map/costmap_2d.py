import numpy as np
import math

class costmap_2d:
    def __init__(self,cells_size_x, cells_size_y, resolution, origin_x, origin_y, default_value):
        self.size_x_ = cells_size_x
        self.size_y_ = cells_size_y
        self.resolution_ = resolution
        self.origin_x_ = origin_x
        self.origin_y_ = origin_y
        self.default_value_ = default_value
        print (self.resolution_)

        self.initMaps(self.size_x_, self.size_y_,self.default_value_)

    def initMaps(self, size_x, size_y,default_value):
        if (size_x>0 and size_y>0):
            self.costmap_ = np.full((size_x, size_y), default_value, int)
        else:
            self.costmap_ = None
            print ("map's size error")

    def resizeMap(self, size_x, size_y, resolution, origin_x, origin_y):
        self.size_x_ = size_x
        self.size_y_ = size_y
        self.resolution_ = resolution
        self.origin_x_ = origin_x
        self.origin_y_ = origin_y

        self.initMaps(self.size_x_, self.size_y_,self.default_value_)
    
    def resetMap(self, x0, y0, xn, yn):
        if xn > self.size_x_:
            xn = self.size_x_
        if yn > self.size_y_:
            yn = self.size_y_
        lenx = xn - x0
        leny = yn - y0
        for i in range(lenx):
            for j in range(leny):
                self.costmap_[i+x0,j+y0] = self.default_value_

    def cellDistance(self, world_dist):
        cells_dist = max(0.0, math.ceil(world_dist / self.resolution_))
        return cells_dist
    
    def getMap(self):
        return self.costmap_
    
    def getCost(self, mx, my):
        if mx < self.size_x_ and my < self.size_y_:
            return self.costmap_[mx, my]
        else:
            return None
        
    def setCost(self, mx, my, cost):
        if mx < self.size_x_ and my < self.size_y_:
            self.costmap_[mx, my] = cost
        else:
            print("Over the border")
    
    def getSizeInCellsX(self):
        return self.size_x_
    
    def getSizeInCellsY(self):
        return self.size_y_

    def getSizeInMetersX(self):
        return (self.size_x_ - 1 + 0.5) * self.resolution_
    
    def getSizeInMetersY(self):
        return (self.size_y_ - 1 + 0.5) * self.resolution_

    def getOriginX(self):
        return self.origin_x_

    def getOriginY(self):
        return self.origin_y_

    def getResolution(self):
        return self.resolution_
    
    def mapToWorld(self, mx, my):
        wx = self.origin_x_ + (mx + 0.5) * self.resolution_
        wy = self.origin_y_ + (my + 0.5) * self.resolution_
        return [wx, wy]


    def worldToMap(self, wx, wy):
        if (wx < self.origin_x_ or wy < self.origin_y_):
            return False

        mx = (int)((wx - self.origin_x_) / self.resolution_)
        my = (int)((wy - self.origin_y_) / self.resolution_)

        if (mx < self.size_x_ and my < self.size_y_):
            return [mx, my]
        else:
            return False

    def worldToMapNoBounds(self, wx, wy):
        mx = (int)((wx - self.origin_x_) / self.resolution_)
        my = (int)((wy - self.origin_y_) / self.resolution_)
        return [mx, my]

    def worldToMapEnforceBounds(self, wx, wy):
        #   Here we avoid doing any math to wx,wy before comparing them to
        #   the bounds, so their values can go out to the max and min values
        #   of double floating point.
        if (wx < self.origin_x_):
            mx = 0
        elif (wx > self.resolution_ * self.size_x_ + self.origin_x_):
            mx = self.size_x_ - 1
        else:
            mx = (int)((wx - self.origin_x_) / self.resolution_)

        if (wy < self.origin_y_):
            my = 0
        elif (wy > self.resolution_ * self.size_y_ + self.origin_y_):
            my = self.size_y_ - 1
        else:
            my = (int)((wy - self.origin_y_) / self.resolution_)
        return [mx, my]
    

    def updateOrigin(self, new_origin_x, new_origin_y):

        cell_ox = int((new_origin_x - self.origin_x_) / self.resolution_)
        cell_oy = int((new_origin_y - self.origin_y_) / self.resolution_)

        new_grid_ox = self.origin_x_ + cell_ox * self.resolution_
        new_grid_oy = self.origin_y_ + cell_oy * self.resolution_
        
        size_x = self.size_x_
        size_y = self.size_y_

        lower_left_x = min(max(cell_ox, 0), size_x)
        lower_left_y = min(max(cell_oy, 0), size_y)
        upper_right_x = min(max(cell_ox + size_x, 0), size_x)
        upper_right_y = min(max(cell_oy + size_y, 0), size_y)

        cell_size_x = upper_right_x - lower_left_x
        cell_size_y = upper_right_y - lower_left_y

        new_costmap = np.full((self.size_x_,self.size_y_), self.default_value_, int)

        for i in range(cell_size_x):
            for j in range(cell_size_y):
                tmp_x1 = lower_left_x - cell_ox + i
                tmp_y1 = lower_left_y - cell_oy + j
                tmp_x2 = lower_left_x + i
                tmp_y2 = lower_left_y + j
                new_costmap[tmp_x1, tmp_y1] = self.costmap_[tmp_x2, tmp_y2]

        self.costmap_ = new_costmap
        self.origin_x_ = new_grid_ox
        self.origin_y_ = new_grid_oy

    def updateSize(self, new_size_x, new_size_y):
        new_costmap = np.full((new_size_x, new_size_y), self.default_value_, int)

        cell_size_x = min(new_size_x, self.size_x_)
        cell_size_y = min(new_size_y, self.size_y_)

        for i in range(cell_size_x):
            for j in range(cell_size_y):
                new_costmap[i,j] = self.costmap_[i,j]
            
        self.costmap_ = new_costmap
        self.size_x_ = new_size_x
        self.size_y_ = new_size_y

    #a = [1,2]

    def convexFillCells(self, lineList):
        lenLine = len(lineList)
        if (lenLine < 2):
            return

        for i in range(lenLine-1):
            self.drawBresenham(lineList[i][0], lineList[i][1], lineList[i+1][0], lineList[i+1][1])
    #def raytraceLine(self, x0, y0, x1, y1):
        
    def drawBresenham(self, x0, y0, x1, y1):
        # x is main axes
        if x0>x1:
            x0,x1=x1,x0
            y0,y1=y1,y0

        delta_x = x1 - x0
        delta_y = y1 - y0
        d = 0
        if delta_x == 0:
            k = 999999999
        else:
            k = delta_y / delta_x

        x = x0
        y = y0
        if k > 1:
            while True:
                if y > y1:
                    break
                self.costmap_[x,y] += 1
                y = y + 1
                d = d + 1 / k
                if d > 0.5:
                    x = x + 1
                    d = d - 1
        elif k > 0:
            while True:
                if x > x1:
                    break
                self.costmap_[x,y] += 1
                x = x + 1
                d = d + k
                if d > 0.5:
                    y = y + 1
                    d = d - 1
        elif k > -1:
            while True:
                if x > x1:
                    break
                self.costmap_[x,y] += 1
                x = x + 1
                d = d - k
                if d > 0.5:
                    y = y - 1
                    d = d - 1
        else:
            while True:
                if y < y1:
                    break
                self.costmap_[x,y] += 1
                y = y - 1
                d = d - 1 / k
                if d > 0.5:
                    x = x + 1
                    d = d - 1
