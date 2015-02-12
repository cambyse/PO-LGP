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

  fx = optimSwitchConfigurations(world_init, world_final, G, 20);
  cout <<"fx=" <<fx <<endl;
  //  world_final.gl().watch();
}

//===========================================================================

void solveProblem(ors::KinematicWorld& world, Graph& symbols){
  runMonteCarlo(symbols);

  ors::KinematicWorld world_end = world;
  Graph G_end = symbols;
  createEndState(world_end, G_end);
//  world_final >>FILE("z.world_fin.kvg");
//  G_final >>FILE("z.symbols_fin.kvg");

  double fx = endStateOptim(world_end, G_end);
  world_end.gl().watch();
  cout <<"fx=" <<fx <<endl;

  return;
  fx = optimSwitchConfigurations(world, world_end, symbols, 20);
  cout <<"fx=" <<fx <<endl;

}

//===========================================================================

void generateRandomProblem(ors::KinematicWorld& world, Graph& symbols){
  Item *CYLIN = symbols["Cylin"];
  Item *BOARD = symbols["Board"];
  Item *DEPTH = symbols["depth"];
  uint n = 15+rnd(50);
  double x=-1.6, y=-1.;
  for(uint i=0;i<n;i++){
    //add an object to the geometry
    ors::Body *b = new ors::Body(world);
    ors::Shape *s = new ors::Shape(world, *b);
    s->cont=true;
    b->X.addRelativeTranslation(x,y,.62);
    //randomize type and size
    if(rnd.uni()<.6){
      s->type = ors::cylinderST;
      s->size[0]=s->size[1]=0.;
      s->size[2]=.2;
      s->size[3]=.05;
      s->name <<"cyl_" <<i;
    }else{
      s->type = ors::boxST;
      s->size[0]=.1 + .3*rnd.uni();
      s->size[1]=.1 + .6*rnd.uni();
      s->size[2]=.02;
      s->size[3]=0.;
      s->name <<"boa_" <<i;
    }
    b->name = s->name;
    //position on grid
    b->X.addRelativeTranslation(0,.5*s->size[1],.5*s->size[2]);
    y += .1 + s->size[1]+s->size[3];
    if(y>1.){ x+=.4; y=-1.; }

    //add symbols
    Item *o = symbols.append<bool>("Object", s->name, new bool(true), true);
    if(s->type==ors::cylinderST){
      symbols.append<bool>(STRINGS_0(), {CYLIN ,o}, new bool(true), true);
    }else{
      symbols.append<bool>(STRINGS_0(), {BOARD, o}, new bool(true), true);
    }
    symbols.append<double>(STRINGS_0(), {DEPTH, o}, new double(0.), true);
  }

  //HACK: move the actionSequence item to the end...
  Item *as = symbols["actionSequence"];
  symbols.ItemL::append(as);
  symbols.ItemL::remove(as->index);
  symbols.index();

  world.calc_fwdPropagateShapeFrames();
}

//===========================================================================

double reward(ors::KinematicWorld& world, Graph& symbols){
  //-- find max depth
  double depth=0.;
  Item *depthSymbol=symbols["depth"];
  for(Item *dep:depthSymbol->parentOf) if(&dep->container==&depthSymbol->container){
    double *d = dep->getValue<double>();
    CHECK(d,"");
    if(*d>depth) depth=*d;
  }

  //-- count supports below
  double supp=0.;
  Item *supportSymbol=symbols["supports"];
  ItemL objs=symbols.getItems("Object");
  for(Item *obj:objs){
    ItemL supporters;
    for(Item *constraint:obj->parentOf){
      if(constraint->parents.N==3 && constraint->parents(0)==supportSymbol && constraint->parents(2)==obj){
        supporters.append(constraint->parents(1));
      }
    }
    supp += .2 * (supporters.N * supporters.N);
  }

  return 10.*depth + supp;
}

//===========================================================================

void coreExperiment(){
//  rnd.clockSeed();

  ors::KinematicWorld world_base("world_base.kvg");
  Graph symbols_base("symbols_base.kvg");

  OpenGL gl;
  gl.add(glStandardScene, 0);
  gl.add(ors::glDrawGraph, &world_base);
  orsDrawJoints=false;
  orsDrawAlpha=1.;

  ofstream fil("data/samples.dat");
  fil <<"experiment #objects MCTS_time lev1_time f_bestEnd lev2_time lev3_time" <<endl;
  for(uint k=0;k<100;k++){
    ors::KinematicWorld world(world_base);
    Graph symbols(symbols_base);

    uint nObjects = world.bodies.N;
    generateRandomProblem(world, symbols);
    nObjects = world.bodies.N - nObjects;
//    world.gl().watch();

    ors::KinematicWorld world_best;
    Graph symbols_best;
    double f_best=0.;

    double MCTS_time=0., lev1_time=0., lev2_time=0., lev3_time=0.;

    uint s;
    for(s=0;s<10;s++){
      ors::KinematicWorld world_sol(world);
      Graph symbols_sol(symbols);
      MT::timerRead(true);
      runMonteCarlo(symbols_sol);
      MCTS_time += MT::timerRead(true);
      createEndState(world_sol, symbols_sol);
      double fx = endStateOptim(world_sol, symbols_sol);
      double rx = reward(world_sol, symbols_sol);
      lev1_time += MT::timerRead(true);
      cout <<"fx=" <<fx <<endl;
      cout <<"reward=" <<rx <<endl;
      if(rx-fx > f_best){
        f_best = rx-fx;
        world_best = world_sol;
        symbols_best = symbols_sol;
      }
      gl.drawers(1).classP= &world_sol;
//      gl.update();
//      world_sol.gl().watch();
    }
    cout <<"BEST:" <<endl;
    world_best >>FILE("z.world_best.kvg");
    symbols_best >>FILE("z.symbols_best.kvg");
    fil <<k <<' ' <<nObjects <<' ' <<MCTS_time/s <<' ' <<lev1_time/s <<' ' <<f_best <<endl;

    gl.drawers(1).classP= &world_best;
//    gl.watch();

//    double f_path = optimSwitchConfigurations(world, world_best, symbols_best, 20);
//    cout <<"f_path=" <<f_path <<endl;

  }
}

//===========================================================================
int main(int argc,char **argv){
  coreExperiment();
//  rnd.clockSeed();
//  generateRandomProblems();
//  return 0;
//  optimizeFinal();
//    optimSwitchConfigurations();
//  testReachable();
//  testMonteCarlo();


  return 0;
}
