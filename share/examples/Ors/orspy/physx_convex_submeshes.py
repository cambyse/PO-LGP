#!/usr/local/env python
# -*- coding: UTF-8 -*-

"""
Test orspy with physx and objects which onsist of convex submeshes.
"""

import orspy
import guipy
import corepy


if __name__ == '__main__':
    graph = orspy.Graph()
    import os
    print os.path.exists("door.ors")
    print os.path.exists("door_model/door-frame.ply")
    print os.path.exists("door_model/door-door.ply")
    graph.init("door.ors")

    robot = graph.getBodyByName("robot")
    # view and physx
    openGL = guipy.OpenGL()
    physxGL = guipy.OpenGL()
    physX = orspy.PhysXInterface()

    orspy.bindOrsToOpenGL(graph, openGL)
    orspy.bindOrsToPhysX(graph, physxGL, physX)

    control = corepy.Vector(0., 0.01, 0)
    for i in range(300):
        robot.X.pos = robot.X.pos + control
        graph.calcBodyFramesFromJoints()

        physX.step()
        openGL.update()
        physxGL.update()
