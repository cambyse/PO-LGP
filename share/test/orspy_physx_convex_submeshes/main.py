"""
Test orspy with physx and objects which onsist of convex submeshes.
"""

# add path of orspy lib to be able to import orspy
import sys
sys.path.append('../../lib/')
import orspy


if __name__ == '__main__':
    graph = orspy.Graph()
    graph.init("world.ors")
    robot = graph.getBodyByName("robot")

    # view and physx
    openGL = orspy.OpenGL()
    physxGL = orspy.OpenGL()
    physX = orspy.PhysXInterface()

    orspy.bindOrsToOpenGL(graph, openGL)
    orspy.bindOrsToPhysX(graph, physxGL, physX)

    control = orspy.Vector(0., 0.01, 0)
    for i in range(300):
        robot.X.pos = robot.X.pos + control
        graph.calcBodyFramesFromJoints()

        physX.step()
        openGL.update()
        physxGL.update()
