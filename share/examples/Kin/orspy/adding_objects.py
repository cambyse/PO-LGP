#!/usr/local/env python
# -*- coding: UTF-8 -*-

"""
Simple orspy test.

Add object during the execution of the program.
"""

import orspy
import corepy
import guipy


if __name__ == '__main__':
    graph = orspy.Graph()
    gl_ors = guipy.OpenGL()
    gl_physx = guipy.OpenGL("physx")
    physx = orspy.PhysXInterface()
    orspy.bindOrsToOpenGL(graph, gl_ors)
    orspy.bindOrsToPhysX(graph, gl_physx, physx)

    bodies = []
    shapes = []
    for i in range(700):

        if i % 30 == 0:
            bodies.append(orspy.Body(graph))
            body = bodies[-1]
            body.type = orspy.dynamicBT
            body.X.pos.setRandom()
            body.X.pos.z += 1

            shapes.append(orspy.Shape(graph, body))
            shape = shapes[-1]
            shape.type = orspy.sphereST
            shape.set_size(.1, .1, .1, .1)

            print "adding object", body
            graph.calcShapeFramesFromBodies()
            # THIS IS IMPOANT
            physx.syncWithOrs()

            # Weird: the following does not work.
            # I guess it has to do with the ors pointers and python handling of
            # labels. the contend of body is overwritten when calling the next
            # line.
            # body = orspy.Body(graph)
            # body.X.pos.x = 3 + i / 200.
            # body.X.pos.y = -2
            # body.X.pos.z = 1
            # body.name = "body_" + str(i)
            # shape = orspy.Shape(graph, body)
            # shape.type = orspy.sphereST
            # shape.set_size(.1, .1, .1, .1)

        graph.calcShapeFramesFromBodies()
        gl_ors.update()
        gl_physx.update()

    print graph
