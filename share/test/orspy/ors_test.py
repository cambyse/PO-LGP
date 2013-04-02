"""
Simple orspy test.

- load an ors file and display a random trajectory
"""

# add path of orspy lib to be able to import orspy
import sys
sys.path.append('../../lib/')
import orspy


if __name__ == '__main__':
    graph = orspy.Graph()
    graph.init("arm7.ors")
    gl = orspy.OpenGL()
    orspy.bindOrsToOpenGL(graph, gl)

    # data
    X = orspy.ArrayDouble()
    V = orspy.ArrayDouble()
    orspy.generateSequence(X, V, graph.getJointStateDimension())

    # follow random trajectory
    for t in range(X.d0):
        graph.setJointState(X[t])
        graph.calcBodyFramesFromJoints()
        gl.timedupdate(0.01)
