#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/constrained.h>
#include <Perception/video.h>
#include <iomanip>

void saveTrajectory(const arr& x, ors::Graph& G, OpenGL& gl) {
  VideoEncoder_libav_simple vid;
  for(uint t=0; t<x.d0; t++) {
    G.setJointState(x[t]);
    G.calcBodyFramesFromJoints();
    gl.update(STRING("step " <<std::setw(3) <<t <<'/' <<x.d0-1).p, true, false);
    flip_image(gl.captureImage);
    vid.addFrame(gl.captureImage);
//    write_ppm(gl.captureImage, STRING("vid/t"<<t<<".ppm"));
  }
  vid.close();
}

arr getSimpleTrajectory(ors::Graph& G){
  MotionProblem P(&G);
  P.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = P.addTaskMap("position",
                   new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.ors->getShapeByName("miniTarget")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  MotionProblemFunction MF(P);
  arr x = P.getInitialization();

  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1., maxStep=1.));
  P.costReport();
  return x;
}


int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  OpenGL gl;
  ors::Graph G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  arr x = getSimpleTrajectory(G);
  arr x2 = reverseTrajectory(x);
  x.append(x2);

  for(uint i=0;i<3;i++)
    displayTrajectory(x, 1, G, gl,"planned trajectory");

  return 0;
}


