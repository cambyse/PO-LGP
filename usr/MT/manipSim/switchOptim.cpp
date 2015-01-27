#include "switchOptim.h"
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/opt-convert.h>
#include <Ors/ors_swift.h>

//===========================================================================

struct SwitchConfigurationProgram:ConstrainedProblemMix{
  ors::KinematicWorld world;
  Graph& symbolicState;
  int verbose;

  MotionProblem MP;
  MotionProblemFunction MPF;

  SwitchConfigurationProgram(ors::KinematicWorld& world_initial, ors::KinematicWorld& world_final, Graph& symbolicState, int verbose)
    : world(world_initial), symbolicState(symbolicState), verbose(verbose), MP(world), MPF(MP){
    ConstrainedProblemMix::operator=( convert_KOrderMarkovFunction_ConstrainedProblemMix(MPF) );

    double posPrec = MT::getParameter<double>("KOMO/moveTo/precision", 1e3);

    //get the actions!
    Item *actionSequence=symbolicState["actionSequence"];
    Graph& actions = actionSequence->kvg();

    //-- set up the MotionProblem
    MP.T=2*actions.N;
    world.swift().initActivations(world);
    MP.world.watch(false);

    Task *t;

    t = MP.addTask("transitions", new TransitionTaskMap(world));
    t->map.order=1; //make this a velocity task!
    t->setCostSpecs(0, MP.T, {0.}, 1e0);

    for(uint i=0;i<actions.N;i++){
      Item *a = actions(i);

      uint endeff_index = world.getShapeByName("hand")->index;
      uint object_index = world.getShapeByName(a->parents(1)->keys(1))->index;
      uint target_index = world.getShapeByName(a->parents(2)->keys(1))->index;

      //pick at time 2*i+1
      t = MP.addTask("pick_pos", new DefaultTaskMap(posDiffTMT, endeff_index, NoVector, object_index, NoVector));
      t->setCostSpecs(2*i+1, 2*i+1, {0.}, posPrec);
      t = MP.addTask("pick_quat", new DefaultTaskMap(quatDiffTMT, endeff_index, NoVector, object_index, NoVector));
      t->setCostSpecs(2*i+1, 2*i+1, {0.}, posPrec);

      //place at time 2*i+2
      ors::Transformation target_X = world_final.shapes(object_index)->X;
      t = MP.addTask("place_pos", new DefaultTaskMap(posTMT, object_index));
      t->setCostSpecs(2*i+2, 2*i+2, ARRAY(target_X.pos), posPrec);
      t = MP.addTask("place_quat", new DefaultTaskMap(quatTMT, object_index));
      t->setCostSpecs(2*i+2, 2*i+2, ARRAY(target_X.rot), posPrec);
    }

/*
    if(colPrec<0){ //interpreted as hard constraint (default)
      t = MP.addTask("collisionConstraints", new CollisionConstraint(margin));
      t->setCostSpecs(0, MP.T, {0.}, 1.);
    }else{ //cost term
      t = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, margin));
      t->setCostSpecs(0, MP.T, {0.}, colPrec);
    }
*/

  }
};

//===========================================================================

double optimSwitchConfigurations(ors::KinematicWorld& world_initial, ors::KinematicWorld& world_final, Graph& symbolicState){
  SwitchConfigurationProgram f(world_initial, world_final, symbolicState, 0);

  arr x = replicate(f.MP.x0, f.MP.T+1); //we initialize with a constant trajectory!
//  rndGauss(x,.01,true); //don't initialize at a singular config

  OptConstrained opt(x, NoArr, f, OPT(verbose=1));
  opt.run();
  f.MP.costReport();
  displayTrajectory(x, 1, f.MP.world, "planned configs", -1);
  return opt.UCP.get_sumOfSquares();
}

//===========================================================================

