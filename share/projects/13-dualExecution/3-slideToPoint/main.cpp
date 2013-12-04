#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/constrained.h>
#include <Perception/video.h>
#include <iomanip>

void saveTrajectory(const arr& x, ors::KinematicWorld& G, OpenGL& gl) {
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

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  OpenGL gl;
  ors::KinematicWorld G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  bool con=true;

  MotionProblem P(&G);
  P.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = P.addTaskMap("position",
                   new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.ors->getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                             ARRAY(0.,0.,0.), 1e1);

  if(con){
    c = P.addTaskMap("collisionConstraints", new CollisionConstraint());
  }else{
    c = P.addTaskMap("collision",
                     new DefaultTaskMap(collTMT, 0, NoVector, 0, NoVector, ARR(.1)));
    P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);
  }

  
  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(P);
  Convert CP(MF);
  UnconstrainedProblem UCP(CP);
  UCP.mu = 10.;

  arr x(MF.get_T()+1,MF.dim_x());
  x.setZero();

  if(con){
    for(uint k=0;k<10;k++){
//      checkAll(CP, x, 1e-4);
      optNewton(x, UCP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
      P.costReport();
      displayTrajectory(x, 1, G, gl,"planned trajectory");
//      saveTrajectory(x, G, gl);
//      UCP.mu *= 10;
      UCP.augmentedLagrangian_LambdaUpdate(x, .9);
    }
  }else{
    for(uint k=0;k<10;k++){
      optNewton(x, CP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1., maxStep=1.));
      P.costReport();
      displayTrajectory(x, 1, G, gl,"planned trajectory");
    }
  }

  return 0;
}


