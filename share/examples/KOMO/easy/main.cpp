#include <Ors/ors.h>
#include <Motion/komo.h>

//===========================================================================

void TEST(Easy){
  ors::KinematicWorld G("test.ors");
  cout <<"configuration space dim=" <<G.q.N <<endl;
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyPR2){
  //NOTE: this uses a 25-DOF whole-body-motion model of the PR2
  ors::KinematicWorld G("model.kvg");
  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  cout <<"configuration space dim=" <<G.q.N <<endl;
  double rand = MT::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(MT::getParameter<uint>("rndSeed", 0));
    rndGauss(G.q,rand,true);
    G.setJointState(G.q);
  }
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyAlign){
  ors::KinematicWorld G("test.ors");
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"), 7); //aligns all 3 axes
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyAlign2){
  ors::KinematicWorld G("test.ors");
  ors::Shape *s = G.getShapeByName("target");
  s->rel.addRelativeRotationDeg(90,1,0,0);
  arr x = moveTo(G, *G.getShapeByName("endeff"), *s, 7, 2); //aligns all 3 axes
  for(uint i=0;i<2;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

//  testEasy();
//  testEasyAlign();
//  testEasyAlign2();
  testEasyPR2();

  return 0;
}


