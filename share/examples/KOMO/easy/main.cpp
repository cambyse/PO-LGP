#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/komo.h>

//===========================================================================

void TEST(Easy){
  ors::KinematicWorld G("test.ors");
  cout <<"configuration space dim=" <<G.q.N <<endl;
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, {}, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyPR2){
  //NOTE: this uses a 25-DOF whole-body-motion model of the PR2
  ors::KinematicWorld G("model.kvg");
  G.meldFixedJoints();
  G.removeUselessBodies();
//  ors::KinematicWorld G2=G;
//  G2.meldFixedJoints();
//  G2.removeUselessBodies();
//  G2 >>FILE("z.ors");
  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  cout <<"configuration space dim=" <<G.q.N <<endl;
  double rand = mlr::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(mlr::getParameter<uint>("rndSeed", 0));
    rndGauss(G.q,rand,true);
    G.setJointState(G.q);
  }
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, {}, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(FinalPosePR2){
  ors::KinematicWorld G("model.kvg");
  G.meldFixedJoints();
  G.removeUselessBodies();
  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  cout <<"configuration space dim=" <<G.q.N <<endl;
  arr x = finalPoseTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  G.setJointState(x.reshape(x.N));
  G.gl().watch();
}

//===========================================================================

void TEST(EasyAlign){
  ors::KinematicWorld G("test.ors");
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"), 7); //aligns all 3 axes
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, {}, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyAlign2){
  ors::KinematicWorld G("test.ors");
  ors::Shape *s = G.getShapeByName("target");
  s->rel.addRelativeRotationDeg(90,1,0,0);
  arr x = moveTo(G, *G.getShapeByName("endeff"), *s, 7, 2); //aligns all 3 axes
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, {}, "planned trajectory", 0.01);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

//  testEasy();
//  testEasyAlign();
//  testEasyAlign2();
  testEasyPR2();
//  testFinalPosePR2();
  return 0;
}


