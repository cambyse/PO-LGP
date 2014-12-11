//#include <pr2/actionMachine.h>
//#include "manipSim.h"
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

void sample();

void RelationalGraph2OrsGraph(ors::KinematicWorld& W, const KeyValueGraph& G){
  W.qdim.clear();
  W.q.clear();
  W.qdot.clear();
  listDelete(W.proxies);
  while(W.joints.N) delete W.joints.last();
  W.isLinkTree=false;
  W.checkConsistency();

//  for(Item *i:G){

//  }

//  //do this first to ensure they have the same indexing
//  for(ors::Body *b:world->bodies){
//    G.append<ors::Body>(STRINGS("body", b->name), b);
//  }

//  for(ors::Body *b:world->bodies){
//    G.append<ors::Transformation>(STRINGS("pose"), ARRAY(G(b->index)), new ors::Transformation(b->X));
////    if(b->ats["ctrlable"]) G.append<bool>(STRINGS("controllable"), ARRAY(G(b->index)), NULL);
//    if(b->ats["canGrasp"]) G.append<bool>(STRINGS("canGrasp"), ARRAY(G(b->index)), NULL);
//    if(b->ats["fixed"])    G.append<bool>(STRINGS("fixed"), ARRAY(G(b->index)), NULL);
//  }

//  for(ors::Joint *j:world->joints){
//    if(j->type==ors::JT_fixed)
//      G.append<bool>(STRINGS("rigid"), ARRAY(G(j->from->index), G(j->to->index)), NULL);
//    if(j->type==ors::JT_transXYPhi)
//      G.append<bool>(STRINGS("support"), ARRAY(G(j->from->index), G(j->to->index)), NULL);
//  }

}

//===========================================================================

void TEST(Reachable){

  KeyValueGraph G;
  ors::KinematicWorld world("model.kvg");

//  G <<FILE("state.kvg");
  RelationalGraph2OrsGraph(world, G);


//  world.checkConsistency();
//  world >>FILE("z.ors");
//  //some optional manipulations
//  world.checkConsistency();
//  world.setShapeNames();
//  world.checkConsistency();
//  world.meldFixedJoints();
//  world.checkConsistency();
//  world >>FILE("z.ors");
//  world.removeUselessBodies();
//  world >>FILE("z.ors");
//  world.topSort();
//  world.makeLinkTree();
//  world.calc_q_from_Q();
//  world.calc_fwdPropagateFrames();
//  world >>FILE("z.ors");

//  if(MT::checkParameter<bool>("cleanOnly")) return;

  for(;;){
    animateConfiguration(world);
    world.gl().watch();
  }
}

//===========================================================================

struct GoalFunction:ConstrainedProblemMix{
  ors::KinematicWorld& world;
  ors::Body *obj, *table;
  arr target;
  GoalFunction(ors::KinematicWorld& world):world(world){
    obj = world.getBodyByName("obj1");
    table = world.getBodyByName("table1");
    target = {-2.,-2.,1.};
    ConstrainedProblemMix::operator=(
      [this](arr& phi, arr& J, TermTypeA& tt, const arr& x) -> void {
        return this -> phi(phi, J, tt, x);
      }
    );
  }
  void phi(arr& phi, arr& phiJ, TermTypeA& tt, const arr& x){
    world.setJointState(x);
    arr y,J;
    world.kinematicsPos(y, J, obj);
//    cout <<"QUERY: pos=" <<y <<endl;
    world.gl().update();

    phi.clear();
    if(&phiJ) phiJ.clear();
    if(&tt) tt.clear();

    //-- cost
    phi.append(y-target);
    if(&tt) tt.append(sumOfSqrTT, y.N);
    if(&phiJ) phiJ.append(J);

    //-- constraints
    arr rel;
    world.kinematicsRelPos(rel, J, obj, NULL, table, NULL);
    phi.append(  rel(0) - (+.5*table->shapes(0)->size[0]-0.05) );
    phi.append( -rel(0) + (-.5*table->shapes(0)->size[0]+0.05) );
    phi.append(  rel(1) - (+.5*table->shapes(0)->size[1]-0.05) );
    phi.append( -rel(1) + (-.5*table->shapes(0)->size[1]+0.05) );
    if(&phiJ){
      phiJ.append( J[0]);
      phiJ.append(-J[0]);
      phiJ.append( J[1]);
      phiJ.append(-J[1]);
    }
    if(&tt) tt.append(ineqTT, 4);

    if(&phiJ) phiJ.reshape(phi.N, x.N);
  }
  virtual uint dim_x(){ return world.getJointStateDimension(); }
  virtual uint dim_g(){ return 4; }
};

void optimizeConfig(){
  ors::KinematicWorld world("model.kvg");

  GoalFunction f(world);

  arr x = world.getJointState();

  checkJacobian(f, x, 1e-4);
  optConstrainedMix(x, NoArr, f, OPT(verbose=1));
  f.world.gl().watch();

//  for(;;){
//    newton.step();
//    f.world.gl().watch();
//    if(newton.stopCriterion) break;
//  }
}

//===========================================================================

int main(int argc,char **argv){

  optimizeConfig();
//  testReachable();
//  sample();

  return 0;
}
