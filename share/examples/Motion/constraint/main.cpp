#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>

void saveTrajectory(const arr& x, ors::KinematicWorld& G, OpenGL& gl) {
  VideoEncoder_libav_simple vid;
  for(uint t=0; t<x.d0; t++) {
    G.setJointState(x[t]);
    gl.update(STRING("step " <<std::setw(3) <<t <<'/' <<x.d0-1).p, true, false);
    flip_image(gl.captureImage);
    vid.addFrame(gl.captureImage);
//    write_ppm(gl.captureImage, STRING("vid/t"<<t<<".ppm"));
  }
  vid.close();
}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));

  bool con=true;

  MotionProblem P(G);
  P.loadTransitionParameters();
  P.makeContactsAttractive=false;

  //-- setup the motion problem
  TaskCost *c;
  c = P.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.world.getBodyByName("target")->X.pos), 1e3);

  c = P.addTask("position_vel", new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  c->map.order=1;
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  if(con){
    c = P.addTask("collisionConstraints", new CollisionConstraint());
    P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1.);
  }else{
    c = P.addTask("collision", new DefaultTaskMap(collTMT, 0, NoVector, 0, NoVector, ARR(.1)));
    P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);
  }

  
  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(P);
  arr x(MF.get_T()+1,MF.dim_x());
  x.setZero();

  Convert CP(MF);
#if 1
  optConstrained(x, P.dualMatrix, CP);
  P.costReport();
  for(uint i=0;i<5;i++) displayTrajectory(x, 1, G, "planned trajectory");
#else
  UnconstrainedProblem UCP(CP);
  UCP.mu = 1.;

  if(con){
    for(uint k=0;k<20;k++){
//      checkAll(CP, x, 1e-4);
      optNewton(x, UCP, OPT(verbose=2, stopIters=100, damping=1., maxStep=1., nonStrict=5));
      P.costReport();
      displayTrajectory(x, 1, G, "planned trajectory");
//      saveTrajectory(x, G, gl);
//      UCP.mu *= 10;
      UCP.aulaUpdate(.9);
    }
  }else{
    for(uint k=0;k<10;k++){
      optNewton(x, CP, OPT(verbose=2, stopIters=100, damping=1., maxStep=1., nonStrict=5));
      P.costReport();
      displayTrajectory(x, 1, G, "planned trajectory");
    }
  }
#endif

  return 0;
}


