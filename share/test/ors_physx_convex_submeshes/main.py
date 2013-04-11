"""
Test orspy with physx and objects which onsist of convex submeshes.
"""

# add path of orspy lib to be able to import orspy
import sys
sys.path.append('../../lib/')
import orspy

import time


if __name__ == '__main__':
    graph = orspy.Graph()
    graph.init("doorSimple.ors")

    openGL = orspy.OpenGL()
    physxGL = orspy.OpenGL()

    physX = orspy.PhysXInterface()
    orspy.bindOrsToOpenGL(graph, openGL)
    orspy.bindOrsToPhysX(graph, physxGL, physX)

    graph.calcBodyFramesFromJoints()

    for i in range(1000):
        graph.calcBodyFramesFromJoints()
        openGL.update()
        physxGL.update()
        print i
