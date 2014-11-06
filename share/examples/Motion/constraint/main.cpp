#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
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

  MotionProblem MP(G);

  //-- setup the motion problem
  Task *c;

  c = MP.addTask("transitions", new TransitionTaskMap(G));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, MP.T, {0.}, 1e0);

  c = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(MP.world.getBodyByName("target")->X.pos), 1e3);

  c = MP.addTask("position_vel", new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  c->map.order=1;
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  if(con){
    c = MP.addTask("collisionConstraints", new CollisionConstraint());
    MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1.);

    Task *sticky = MP.addTask("collisionStickiness", new ConstraintStickiness(c->map));
    sticky->setCostSpecs(0, MP.T, {0.}, 1.e1);
  }else{
    c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {}, {.1}));
    MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);
  }

  
  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x(MF.get_T()+1,MF.dim_x());
  x.setZero();

  Convert CP(MF);
  for(uint k=0;k<1;k++){
    optConstrained(x, MP.dualMatrix, CP);
    MP.costReport();
    for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory");
  }

  return 0;
}


