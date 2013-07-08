"""
Ors PhysX test
==============

TODO memory management does not work proprely. I get a segfault when the
     program terminates. Double free memory.

TODO need more swig wrapper :(
"""

import sys
sys.path.append('../../lib/')

import orspy


if __name__ == '__main__':
    graph = orspy.Graph()
    graph.clear()
    for k in range(1):
        # TODO this leads to a segfault. find out why!
        body = orspy.Body(graph)
        # body = orspy.Body()
        body.X.setRandom()
        body.X.pos.z += 1.
        body.set_name("rndSphere_" + str(k))
        print body

        # TODO this leads to a segfault. find out why!
        # shape = orspy.Shape(graph, body)
        shape = orspy.Shape()
        shape.type = orspy.boxST
        shape.set_size(.1, .1, .1, .1)
        print shape

        print "done"
        # graph.addObject(body)
    print "after loop"

    # visGL = orspy.OpenGL()
    # visPhysX = orspy.OpenGL()
    # physX = orspy.PhysXInterface()

    # orspy.bindOrsToOpenGL(graph, visGL)
    # orspy.bindOrsToPhysX(graph, visPhysX, physX)

    # for t in range(500):
        # physX.step()
        # visGL.update()
        # visPhysX.update()
