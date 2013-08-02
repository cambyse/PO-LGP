#!/usr/local/env python
# -*- coding: UTF-8 -*-

"""
Simple orspy test.

Add object during the execution of the program.
"""

import orspy
import corepy


if __name__ == '__main__':
    graph = orspy.Graph()
    gl = corepy.OpenGL()
    orspy.bindOrsToOpenGL(graph, gl)

    bodies = []
    shapes = []
    for i in range(700):

        if i % 50 == 0:
            print "adding object"
            bodies.append(orspy.Body(graph))
            body = bodies[-1]

            body.X.pos.setRandom()
            body.X.pos.z += 1
            body.name = "body_" + str(i)

            shapes.append(orspy.Shape(graph, body))
            shape = shapes[-1]
            shape.type = orspy.sphereST
            shape.set_size(.1, .1, .1, .1)

            print body
            print

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
        gl.update()

    print graph
