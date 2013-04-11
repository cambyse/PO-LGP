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
    print "gl created"

    physX = orspy.PhysXInterface()
    print "physx created"
    orspy.bindOrsToOpenGL(graph, openGL)
    print "after bind to gl"
    orspy.bindOrsToPhysX(graph, physxGL, physX)
    print "after bind to physx"

    graph.calcBodyFramesFromJoints()
    print "after calc"

    for i in range(1000):
        print "in for loop"
        graph.calcBodyFramesFromJoints()
        openGL.update()
        physxGL.update()
        print i
