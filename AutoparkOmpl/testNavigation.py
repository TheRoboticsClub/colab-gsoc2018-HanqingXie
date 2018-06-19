from navigation.map.costmap_2d import costmap_2d



map_test = costmap_2d(10,10,0.1,0,0,0)
costmap = map_test.getMap()
print (costmap)

line = [[0,0],[5,4],[9,6]]
map_test.convexFillCells(line)

costmap = map_test.getMap()
print (costmap)