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

    # follow random trajectory
    # data
    # X = orspy.ArrayDouble()
    # V = orspy.ArrayDouble()
    # orspy.generateSequence(X, V, graph.getJointStateDimension())
    # for t in range(X.d0):
    #     graph.setJointState(X[t])
    #     graph.calcBodyFramesFromJoints()
    #     gl.timedupdate(0.01)

    # set joint state directly (with a python list)
    jsd = graph.getJointStateDimension()
    for i in range(700):
        jointState = [0.01 * i] * jsd
        graph.setJointStateList(jointState)
        graph.calcBodyFramesFromJoints()
        gl.update()
