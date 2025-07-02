#=============================================================================
#     Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
#                      Author: Gonzalo Mier
#                         BSD-3 License
#=============================================================================

import math
import fields2cover as f2c
import os

p1 = f2c.Point(-5.0,-5.0)
p2 = f2c.Point(-5.0,5.0)
p3 = f2c.Point(5.0,5.0)
p4 = f2c.Point(5.0,-5.0)

field = f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint([
    p1, p2, p3, p4, p1]))))

robot = f2c.Robot(0.2, 1.0)
robot.setMinTurningRadius(0.1)
robot.setCruiseVel(1.0)
robot.setTurnVel(1.0)
robot.setMaxDiffCurv(0.5)

const_hl = f2c.HG_Const_gen()
no_hl = const_hl.generateHeadlands(field, 2.0 * robot.getWidth())
bf = f2c.SG_BruteForce()
swaths = bf.generateSwaths(math.pi/2, robot.getCovWidth(), no_hl.getGeometry(0))
#boustrophedon_sorter = f2c.RP_Boustrophedon()
#swaths = boustrophedon_sorter.genSortedSwaths(swaths)

swaths_by_cells = f2c.SwathsByCells()
swaths_by_cells.push_back(swaths)

# Use RoutePlannerBase to set the start point explicitly
route_planner = f2c.RP_RoutePlannerBase()
start_pt = f2c.Point(0.0, 0.0)
route_planner.setStartAndEndPoint(start_pt)
route = route_planner.genRoute(no_hl, swaths_by_cells)

path_planner = f2c.PP_PathPlanning()
dubins = f2c.PP_DubinsCurves()
path = path_planner.planPath(robot, route, dubins)

f2c.Visualizer.figure()
f2c.Visualizer.plot(field)
f2c.Visualizer.plot(no_hl)
f2c.Visualizer.plot(path)
f2c.Visualizer.axis_equal()
f2c.Visualizer.save("Tutorial_9_1.png")


