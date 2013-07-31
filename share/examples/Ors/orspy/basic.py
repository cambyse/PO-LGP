"""
Simple orspy test.

- load an ors file and display a random trajectory
"""

# add path of orspy lib to be able to import orspy
import orspy
import corepy


if __name__ == '__main__':
    graph = orspy.Graph()
    graph.init("arm7.ors")
    gl = corepy.OpenGL()
    orspy.bindOrsToOpenGL(graph, gl)

    jsd = graph.getJointStateDimension()
    for i in range(700):
        # set joint state directly (with a python list)
        jointState = [0.01 * i] * jsd
        graph.setJointStateList(jointState)

        graph.calcBodyFramesFromJoints()
        gl.update()
