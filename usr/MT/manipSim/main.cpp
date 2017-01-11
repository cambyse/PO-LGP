//#include <RosCom/actionMachine.h>
//#include "manipSim.h"
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Optim/optimization.h>

#include "endStateOptim.h"
#include "switchOptim.h"
#include "blackbox.h"

//===========================================================================

void sample();
void testMonteCarlo();

void RelationalGraph2OrsGraph(mlr::KinematicWorld& W, const Graph& G){
  W.qdim.clear();
  W.q.clear();
  W.qdot.clear();
  listDelete(W.proxies);
  while(W.joints.N) delete W.joints.last();
  W.isLinkTree=false;
  W.checkConsistency();

  //  for(Node *i:G){

  //  }

  //  //do this first to ensure they have the same indexing
  //  for(mlr::Body *b:world->bodies){
  //    G.newNode<mlr::Body>({"body", b->name}, b);
  //  }

  //  for(mlr::Body *b:world->bodies){
  //    G.newNode<mlr::Transformation>({"pose"}, ARRAY(G(b->index)), new mlr::Transformation(b->X));
  ////    if(b->ats["ctrlable"]) G.newNode<bool>({"controllable"}, ARRAY(G(b->index)), NULL);
  //    if(b->ats["canGrasp"]) G.newNode<bool>({"canGrasp"}, ARRAY(G(b->index)), NULL);
  //    if(b->ats["fixed"])    G.newNode<bool>({"fixed"}, ARRAY(G(b->index)), NULL);
  //  }

  //  for(mlr::Joint *j:world->joints){
  //    if(j->type==mlr::JT_rigid)
  //      G.newNode<bool>({"rigid"}, ARRAY(G(j->from->index), G(j->to->index)), NULL);
  //    if(j->type==mlr::JT_transXYPhi)
  //      G.newNode<bool>({"support"}, ARRAY(G(j->from->index), G(j->to->index)), NULL);
  //  }

}

//===========================================================================

void optimizeFinal(){
  mlr::KinematicWorld world("world_finalExample.kvg");
  Graph G("symbols_finalExample.kvg");

  world >>FILE("z.kvg");

  double fx = endStateOptim(world, G);
  cout <<"fx=" <<fx <<endl;
  world.gl().watch();

}

//===========================================================================

void optimSwitchConfigurations(){
  mlr::KinematicWorld world_init("world.kvg");
  Graph G("symbols.kvg");

  mlr::KinematicWorld world_final = world_init;
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

void solveProblem(mlr::KinematicWorld& world, Graph& symbols){
  runMonteCarlo(symbols);

  mlr::KinematicWorld world_end = world;
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

void generateRandomProblem(mlr::KinematicWorld& world, Graph& symbols){
  symbols.checkConsistency();
  Node *CYLIN = symbols["Cylin"];
  Node *BOARD = symbols["Board"];
  Node *DEPTH = symbols["depth"];
  Graph& state = symbols["STATE"]->graph();

  uint n = 10+rnd(20);
  double x=-1.6, y=-1.;
  for(uint i=0;i<n;i++){
    //add an object to the geometry
    mlr::Body *b = new mlr::Body(world);
    mlr::Shape *s = new mlr::Shape(world, *b);
    s->cont=true;
    b->X.addRelativeTranslation(x,y,.62);
    //randomize type and size
    if(rnd.uni()<.6){
      s->type = mlr::cylinderST;
      s->size[0]=s->size[1]=0.;
      s->size[2]=.2;
      s->size[3]=.05;
      s->name <<"cyl_" <<i;
    }else{
      s->type = mlr::boxST;
      s->size[0]=.1 + .3*rnd.uni();
      s->size[1]=.1 + .6*rnd.uni();
      s->size[2]=.02;
      s->size[3]=0.;
      s->name <<"boa_" <<i;
    }
    b->name = s->name;
    //position on grid
    b->X.addRelativeTranslation(0, .5*s->size[1], .5*s->size[2]);
    y += .1 + s->size[1]+s->size[3];
    if(y>1.){ x+=.4; y=-1.; }

    //add symbols
    Node *o = symbols.newNode<bool>({"Object", s->name}, {}, true);
    if(s->type==mlr::cylinderST){
      state.newNode<bool>({}, {CYLIN ,o}, true);
    }else{
      state.newNode<bool>({}, {BOARD, o}, true);
    }
    state.newNode<double>({}, {DEPTH, o}, 0.);
  }

  symbols.checkConsistency();

  //HACK: move the actionSequence item to the end...
  Node *ss = symbols["STATE"];
  symbols.NodeL::append(ss);
  symbols.NodeL::remove(ss->index);
  symbols.index();

  Node *as = symbols["actionSequence"];
  symbols.NodeL::append(as);
  symbols.NodeL::remove(as->index);
  symbols.index();

  world.calc_fwdPropagateShapeFrames();
}

//===========================================================================

double reward(mlr::KinematicWorld& world, Graph& symbols){
  //-- find max depth
  double depth=0.;
  Node *depthSymbol=symbols["depth"];
  Graph& state =symbols["STATE"]->graph();

  for(Node *dep:depthSymbol->parentOf) if(&dep->container==&state){
    double d = dep->get<double>();
      if(d>depth) depth=d;
  }

  //-- count supports below
  double supp=0.;
  Node *supportSymbol=symbols["supports"];
  NodeL objs=symbols.getNodes("Object");
  for(Node *obj:objs){
    NodeL supporters;
    for(Node *constraint:obj->parentOf){
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

  mlr::KinematicWorld world_base("world_base.kvg");
  Graph symbols_base("symbols_base.kvg");

  OpenGL gl;
  gl.add(glStandardScene, 0);
  gl.add(mlr::glDrawGraph, &world_base);
  orsDrawJoints=false;
  orsDrawAlpha=1.;

  ofstream fil("data/samples.dat");
  fil <<"experiment #objects MCTS_time lev1_time f_bestEnd lev2_time lev3_time" <<endl;
  for(uint k=0;k<50;k++){
    mlr::KinematicWorld world(world_base);
    Graph symbols(symbols_base);

    uint nObjects = world.bodies.N;
    generateRandomProblem(world, symbols);
    nObjects = world.bodies.N - nObjects;
//    world .gl().watch();

    mlr::KinematicWorld world_best, world_display;
    Graph symbols_best;
    double f_best=0.;

    double MCTS_time=0., lev1_time=0., lev2_time=0., lev3_time=0.;

    uint s;
    for(s=0;s<200;s++){
      mlr::KinematicWorld world_sol(world);
      Graph symbols_sol(symbols);
      mlr::timerRead(true);
      runMonteCarlo(symbols_sol);
      MCTS_time += mlr::timerRead(true);
      createEndState(world_sol, symbols_sol);
      double fx = endStateOptim(world_sol, symbols_sol);
      double rx = reward(world_sol, symbols_sol);
      lev1_time += mlr::timerRead(true);
      cout <<"fx=" <<fx <<endl;
      cout <<"reward=" <<rx <<endl;
      if(rx-fx > f_best){
        f_best = rx-fx;
        world_best = world_sol;
        symbols_best = symbols_sol;
      }
//      gl.drawers(1).classP= &world_sol;
      world_display=world_sol;
      world_display.gl().update();
//      gl.update();
//      world_sol.gl().update();
    }
    cout <<"BEST:" <<endl;
    world_best >>FILE("z.world_best.kvg");
    symbols_best >>FILE("z.symbols_best.kvg");

    world_display=world_best;
    world_display.gl().update();

//    gl.drawers(1).classP= &world_best;
    gl.update();
//    gl.watch();

    mlr::timerRead(true);
    double f_path = optimSwitchConfigurations(world, world_best, symbols_best, 20);
    lev2_time = mlr::timerRead(true);
    cout <<"f_path=" <<f_path <<endl;

    fil <<k <<' ' <<nObjects <<' ' <<MCTS_time/s <<' ' <<lev1_time/s <<' ' <<f_best <<' ' <<lev2_time <<endl;

  }
}

//===========================================================================
int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);
//  rnd.clockSeed();
  rnd.seed(mlr::getParameter<int>("seed",0));

  coreExperiment();
//  generateRandomProblems();
//  return 0;
//  optimizeFinal();
//    optimSwitchConfigurations();
//  testReachable();
//  testMonteCarlo();


  return 0;
}
