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
  uint microSteps;

  MotionProblem MP;
  MotionProblemFunction MPF;

  SwitchConfigurationProgram(ors::KinematicWorld& world_initial, ors::KinematicWorld& world_final, Graph& symbolicState, int verbose)
    : world(world_initial), symbolicState(symbolicState), verbose(verbose), MP(world), MPF(MP){
    ConstrainedProblemMix::operator=( convert_KOrderMarkovFunction_ConstrainedProblemMix(MPF) );

    microSteps = 20;
    double posPrec = MT::getParameter<double>("LGP/precision", 1e3);
    double colPrec = MT::getParameter<double>("LGP/collisionPrecision", -1e0);
    double margin = MT::getParameter<double>("LGP/collisionMargin", .05);

    //get the actions!
    Item *actionSequence=symbolicState["actionSequence"];
    Graph& actions = actionSequence->kvg();

    //-- set up the MotionProblem
    MP.T=2*actions.N*microSteps;
    world.swift().initActivations(world);
    MP.world.watch(false);

    //-- decide on pickAndPlace times
    uintA tPick(actions.N), tPlace(actions.N);
    for(uint i=0;i<actions.N;i++){
      tPick(i) = (2*i+1)*microSteps;
      tPlace(i) = (2*i+2)*microSteps;
    }

    //-- transitions
    Task *t;
    t = MP.addTask("transitions", new TransitionTaskMap(world));
    t->map.order=2;
    t->setCostSpecs(0, MP.T, {0.}, 1e0);

//    if(colPrec<0){ //interpreted as hard constraint (default)
//      t = MP.addTask("collisionConstraints", new CollisionConstraint(margin));
//      t->setCostSpecs(0, MP.T, {0.}, 1.);
//    }else{ //cost term
//      t = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, margin));
//      t->setCostSpecs(0, MP.T, {0.}, colPrec);
//    }

    for(uint i=0;i<actions.N;i++){
      Item *a = actions(i);

      uint endeff_index = world.getShapeByName("graspRef")->index;
      uint object_index = world.getShapeByName(a->parents(1)->keys(1))->index;
      uint target_index = world.getShapeByName(a->parents(2)->keys(1))->index;

      //pick at time 2*i+1
      ors::GraphOperator *op_pick = new ors::GraphOperator();
      op_pick->symbol = ors::GraphOperator::addRigid;
      op_pick->timeOfApplication = tPick(i)+1;
      op_pick->fromId = world.shapes(endeff_index)->body->index;
      op_pick->toId = world.shapes(object_index)->body->index;
      world.operators.append(op_pick);

      t = MP.addTask("pick_pos", new DefaultTaskMap(posDiffTMT, endeff_index, NoVector, object_index, NoVector));
      t->setCostSpecs(tPick(i), tPick(i), {0.}, posPrec);
      t = MP.addTask("pick_quat", new DefaultTaskMap(quatDiffTMT, endeff_index, NoVector, object_index, NoVector));
      t->setCostSpecs(tPick(i), tPick(i), {0.}, posPrec);

      t = MP.addTask("pick_pos_vel", new DefaultTaskMap(posDiffTMT, endeff_index, NoVector, object_index, NoVector));
      t->map.order=1;
      t->setCostSpecs(tPick(i), tPick(i), {0.}, posPrec);
      t = MP.addTask("pick_quat_vel", new DefaultTaskMap(quatDiffTMT, endeff_index, NoVector, object_index, NoVector));
      t->map.order=1;
      t->setCostSpecs(tPick(i), tPick(i), {0.}, posPrec);

      t = MP.addTask("pick_pos_vel", new DefaultTaskMap(posDiffTMT, endeff_index));
      t->map.order=1;
      t->setCostSpecs(tPick(i)+2, tPick(i)+2, {0.,0.,.1}, posPrec);

      //place at time 2*i+2
      ors::GraphOperator *op_place = new ors::GraphOperator();
      op_place->symbol = ors::GraphOperator::deleteJoint;
      op_place->timeOfApplication = tPlace(i)+1;
      op_place->fromId = world.shapes(endeff_index)->body->index;
      op_place->toId = world.shapes(object_index)->body->index;
      world.operators.append(op_place);

      ors::Transformation target_X = world_final.shapes(object_index)->X;
      t = MP.addTask("place_pos", new DefaultTaskMap(posTMT, object_index));
      t->setCostSpecs(tPlace(i), tPlace(i), ARRAY(target_X.pos), posPrec);
      t = MP.addTask("place_quat", new DefaultTaskMap(quatTMT, object_index));
      t->setCostSpecs(tPlace(i), tPlace(i), ARRAY(target_X.rot), posPrec);

      t = MP.addTask("place_pos_vel", new DefaultTaskMap(posTMT, object_index));
      t->map.order=1;
      t->setCostSpecs(tPlace(i), tPlace(i), {0.}, posPrec);
      t = MP.addTask("place_quat_vel", new DefaultTaskMap(quatTMT, object_index));
      t->map.order=1;
      t->setCostSpecs(tPlace(i), tPlace(i), {0.}, posPrec);

      t = MP.addTask("place_pos_vel", new DefaultTaskMap(posTMT, object_index));
      t->map.order=1;
      t->setCostSpecs(tPlace(i)-2, tPlace(i)-2, {0.,0.,-.1}, posPrec);
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

  OptConstrained opt(x, NoArr, f, OPT(verbose=1, damping = 1e-2));
  opt.run();
  f.MP.costReport();
  displayTrajectory(x, 1, f.MP.world, "planned configs", 0);
  return opt.UCP.get_sumOfSquares();
}

//===========================================================================

