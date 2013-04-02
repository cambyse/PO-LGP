"""
Simple orspy test.

- load an ors file and display a random trajectory

TODO move this to a better place

TODO how to handle python wrappers?
     There needs to be the *.py file of the module and the *.so file in the
     path.  Currently the libMT.so is copied by hand to the folder where the
     *.py file is.  Also the *.py file is placed where the make file is.  Maybe
     we should move the *.py to the lib folder and then add the path in the
     python scripts? Assuming the python files are in share/test/X/* and the
     makefile was changed to put all file in lib/, the procedure would look
     like this:
>>> import sys
>>> sys.path.append('../../lib/')
>>> import ors
>>> # do stuff with ors
>>> g = ors.Graph()
>>> # ...
"""

import ors


if __name__ == '__main__':
    graph = ors.Graph()
    graph.init("arm7.ors")
    gl = ors.OpenGL()
    ors.bindOrsToOpenGL(graph, gl)

    # data
    X = ors.ArrayDouble()
    V = ors.ArrayDouble()
    ors.generateSequence(X, V, graph.getJointStateDimension())

    # follow random trajectory
    for t in range(X.d0):
        graph.setJointState(X[t])
        graph.calcBodyFramesFromJoints()
        gl.timedupdate(0.01)
