//#include <pr2/actionMachine.h>
//#include "manipSim.h"
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

//===========================================================================

void sample();
void testMonteCarlo();

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
  Graph& symbolicState;
//  ors::Body *obj, *table;
//  arr target;
  GoalFunction(ors::KinematicWorld& world, Graph& symbolicState):world(world), symbolicState(symbolicState){
//    obj = world.getBodyByName("obj1");
//    table = world.getBodyByName("table1");
//    target = {-2.,-2.,1.};
    ConstrainedProblemMix::operator=(
      [this](arr& phi, arr& J, TermTypeA& tt, const arr& x) -> void {
        return this -> phi(phi, J, tt, x);
      }
    );
  }
  void phi(arr& phi, arr& phiJ, TermTypeA& tt, const arr& x){
    world.setJointState(x);
    world.gl().watch();

    phi.clear();
    if(&phiJ) phiJ.clear();
    if(&tt) tt.clear();

    //-- cost
//    arr y,J;
//    world.kinematicsPos(y, J, obj);
////    cout <<"QUERY: pos=" <<y <<endl;
//    phi.append(y-target);
//    if(&tt) tt.append(sumOfSqrTT, y.N);
//    if(&phiJ) phiJ.append(J);

    //-- support symbols -> overlap constraints
    Item *support=symbolicState["supports"];
    for(Item *constraint:support->parentOf){
      ors::Body *b1=world.getBodyByName(constraint->parents(1)->keys(1));
      ors::Body *b2=world.getBodyByName(constraint->parents(2)->keys(1));
      arr y,J;
      world.kinematicsRelPos(y, J, b1, NULL, b2, NULL);
      arr range(3);
      range(0) = .5*fabs(b1->shapes(0)->size[0] - b2->shapes(0)->size[0]);
      range(1) = .5*fabs(b1->shapes(0)->size[1] - b2->shapes(0)->size[1]);
      range(2)=0.;
      cout <<y <<range
          <<y-range <<-y-range
         <<"\n 10=" <<b1->shapes(0)->size[0]
        <<" 20=" <<b2->shapes(0)->size[0]
       <<" 11=" <<b1->shapes(0)->size[1]
      <<" 21=" <<b2->shapes(0)->size[1]
        <<endl;
      phi.append(  y(0) - range(0) );
      phi.append( -y(0) - range(0) );
      phi.append(  y(1) - range(1) );
      phi.append( -y(1) - range(1) );
      if(&phiJ){
        phiJ.append( J[0]);
        phiJ.append(-J[0]);
        phiJ.append( J[1]);
        phiJ.append(-J[1]);
      }
      if(&tt) tt.append(ineqTT, 4);
    }

    //-- support -> maximize distances
    ItemL objs=symbolicState.getItems("Object");
    for(Item *obj:objs){
      ItemL supporters;
      for(Item *constraint:obj->parentOf){
        if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(2)==obj){
          supporters.append(constraint->parents(1));
        }
      }
      cout <<"Object" <<*obj <<" is supported by "; listWrite(supporters, cout); cout <<endl;
      if(supporters.N==2){
        ors::Body *b1=world.getBodyByName(supporters(0)->keys(1));
        ors::Body *b2=world.getBodyByName(supporters(1)->keys(1));
        arr y1,y2,J1,J2;
        world.kinematicsPos(y1, J1, b1);
        world.kinematicsPos(y2, J2, b2);
        arr y = y1-y2;
        double d = length(y);
        arr normal = y/d;
        phi.append( 1.-d );
        if(&phiJ){
          arr J = ~normal*(-J1+J2);
          phiJ.append( J );
        }
        if(&tt) tt.append(sumOfSqrTT, 1);
      }
    }

    if(&phiJ) phiJ.reshape(phi.N, x.N);
  }
  virtual uint dim_x(){ return world.getJointStateDimension(); }
  virtual uint dim_g(){ return 4; }
};

//===========================================================================

void optimizeFinal(){
  ors::KinematicWorld world("model.kvg");
  Graph G("final.kvg");

  GoalFunction f(world, G);

  arr x = world.getJointState();

//  checkJacobianCP(f, x, 1e-4);
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

//  optimizeFinal();
//  testReachable();
  testMonteCarlo();


  return 0;
}
