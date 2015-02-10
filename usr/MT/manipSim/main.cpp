//#include <pr2/actionMachine.h>
//#include "manipSim.h"
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

#include "endStateOptim.h"
#include "switchOptim.h"
#include "search.h"

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

void optimizeFinal(){
  ors::KinematicWorld world("world_finalExample.kvg");
  Graph G("symbols_finalExample.kvg");

  world >>FILE("z.kvg");

  double fx = endStateOptim(world, G);
  cout <<"fx=" <<fx <<endl;
  world.gl().watch();

}

//===========================================================================

void optimSwitchConfigurations(){
  ors::KinematicWorld world_init("world.kvg");
  Graph G("symbols.kvg");

  ors::KinematicWorld world_final = world_init;
  Graph G_final = G;
  createEndState(world_final, G_final);
  world_final >>FILE("z.world_final.kvg");
  G_final >>FILE("z.symbols_final.kvg");


  double fx = endStateOptim(world_final, G_final);
  cout <<"fx=" <<fx <<endl;

  fx = optimSwitchConfigurations(world_init, world_final, G);
  cout <<"fx=" <<fx <<endl;
//  world_final.gl().watch();
}

//===========================================================================

void solveProblem(ors::KinematicWorld& world, Graph& symbols){
  runMonteCarlo(symbols);

  ors::KinematicWorld world_final = world;
  Graph G_final = symbols;
  createEndState(world_final, G_final);
  world_final >>FILE("z.world_fin.kvg");
  G_final >>FILE("z.symbols_fin.kvg");

  double fx = endStateOptim(world_final, G_final);
  world_final.gl().watch();
  cout <<"fx=" <<fx <<endl;

  return;
  fx = optimSwitchConfigurations(world, world_final, symbols);
  cout <<"fx=" <<fx <<endl;

}

//===========================================================================

void generateRandomProblems(){
  ors::KinematicWorld world_base("world_base.kvg");
  Graph symbols_base("symbols_base.kvg");

  for(uint k=0;k<10;k++){
    ors::KinematicWorld world(world_base);
    Graph symbols(symbols_base);
    uint n = 5+rnd(30);
    double x=-1.6, y=-1.;
    for(uint i=0;i<n;i++){
      //add an object to the geometry
      ors::Body *b = new ors::Body(world);
      ors::Shape *s = new ors::Shape(world, *b);
      b->name <<"obj_" <<i;
      s->name = b->name;
      s->cont=true;
      b->X.addRelativeTranslation(x,y,.62);
      //randomize type and size
      if(rnd.uni()<.6){
        s->type = ors::cylinderST;
        s->size[1]=.1;
        s->size[2]=.2;
        s->size[3]=.05;
      }else{
        s->type = ors::boxST;
        s->size[0]=.1 + .3*rnd.uni();
        s->size[1]=.1 + .6*rnd.uni();
        s->size[2]=.02;
      }
      //position on grid
      b->X.addRelativeTranslation(0,.5*s->size[1],.5*s->size[2]);
      y += .1 + s->size[1];
      if(y>1.){ x+=.4; y=-1.; }

      //add symbols
      symbols.append<bool>("Object", s->name, new bool(true), true);
    }

    //HACK: move the actionSequence item to the end...
    Item *as = symbols["actionSequence"];
    symbols.ItemL::append(as);
    symbols.ItemL::remove(as->index);
    symbols.index();

    cout <<symbols <<endl;
    world.calc_fwdPropagateShapeFrames();
    world.gl().watch();
    solveProblem(world, symbols);
  }
}

//===========================================================================

int main(int argc,char **argv){
  rnd.clockSeed();
//  generateRandomProblems();
//  return 0;
  optimizeFinal();
//  optimSwitchConfigurations();
//  testReachable();
//  testMonteCarlo();


  return 0;
}
