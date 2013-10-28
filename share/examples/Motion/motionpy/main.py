import orspy as ors
import corepy as core
import motionpy as motion
import optimpy as optim
import numpy as np

gl = core.OpenGL()
G = ors.Graph()
G.init("test.ors")
ors.bindOrsToOpenGL(G, gl)

P = motion.MotionProblem(G)
P.loadTransitionParameters()

t = core.Transformation()
t.setText("<t(0 0 .2)>")

c = P.addDefaultTaskMap_Bodies("position", motion.posTMT, "endeff", t)
P.setInterpolatingCosts(c, motion.MotionProblem.constFinalMid,
                        core.ARRAY(P.ors.getBodyByName("target").X.pos), 1e3, 
                        np.array([0, 0, 0]), 1e-3)
P.setInterpolatingVelCosts(c, motion.MotionProblem.constFinalMid,
                           np.array([0, -1, 0]), 1e-0,
                           np.array([0, 0, 0]), 0)

c = P.addDefaultTaskMap("collision", motion.collTMT , 0, core.Transformation_Id,
                        0, core.Transformation_Id, np.array([.1]))
P.setInterpolatingCosts(c, motion.MotionProblem.constFinalMid, np.array([0.]),
                        1e-0)

c = P.addDefaultTaskMap("qitself", motion.qItselfTMT, 0, core.Transformation_Id,
                        0, core.Transformation_Id, np.array([0]))
P.setInterpolatingCosts(c, motion.MotionProblem.constFinalMid, np.array([0.]),
                        1e-4)

F = motion.MotionProblemFunction(P)
T = F.get_T()
k = F.get_k()
n = F.dim_x()

print("Problem parameters:")
print("T=" + str(T))
print("k=" + str(k))
print("n=" + str(n))

x = core.zeros(T+1, n)

print("fx = " + str(optim.evaluateVF(optim.Convert(F).asVectorFunction(), x)))

opt = optim.OptOptions();
opt.verbose = 2
opt.stopIters = 40
opt.useAdaptiveDamping = False
opt.damping = 1e-0
opt.maxStep = 1.

for k in range(0,1):
    (r, x) = optim.optNewton(x, optim.Convert(F).asScalarFunction(), opt)
    ors.displayTrajectory(x, 1, G, gl, "planned trajectory")
