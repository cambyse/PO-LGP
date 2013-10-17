#!/usr/local/env python
# -*- coding: UTF-8 -*-

"""
Test orspy with physx and objects which onsist of convex submeshes.
"""

import corepy
import orspy


if __name__ == '__main__':
    graph = orspy.Graph()
    import os
    print os.path.exists("door.ors")
    print os.path.exists("door_model/door-frame.ply")
    print os.path.exists("door_model/door-door.ply")
    graph.init("door.ors")

    import sys
    sys.exit()
    robot = graph.getBodyByName("robot")
    # view and physx
    openGL = corepy.OpenGL()
    physxGL = corepy.OpenGL()
    physX = orspy.PhysXInterface()

    orspy.bindOrsToOpenGL(graph, openGL)
    orspy.bindOrsToPhysX(graph, physxGL, physX)

    control = orspy.Vector(0., 0.01, 0)
    for i in range(300):
        robot.X.pos = robot.X.pos + control
        graph.calcBodyFramesFromJoints()

        physX.step()
        openGL.update()
        physxGL.update()
