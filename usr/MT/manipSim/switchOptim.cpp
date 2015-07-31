#include "switchOptim.h"
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/opt-convert.h>
#include <Ors/ors_swift.h>

//===========================================================================

struct SwitchConfigurationProgram:ConstrainedProblemMix{
  ors::KinematicWorld world;
  Graph& symbolicState;
  uint microSteps;
  int verbose;

  MotionProblem MP;
  MotionProblemFunction MPF;

  SwitchConfigurationProgram(ors::KinematicWorld& world_initial, ors::KinematicWorld& world_final,
                             Graph& symbolicState,
                             uint microSteps,
                             int verbose)
    : world(world_initial), symbolicState(symbolicState), microSteps(microSteps), verbose(verbose), MP(world), MPF(MP){
    ConstrainedProblemMix::operator=( convert_KOrderMarkovFunction_ConstrainedProblemMix(MPF) );

    double posPrec = MT::getParameter<double>("LGP/precision", 1e3);
    double colPrec = MT::getParameter<double>("LGP/collisionPrecision", -1e0);
    double margin = MT::getParameter<double>("LGP/collisionMargin", .05);

    //get the actions!
    Node *actionSequence=symbolicState["actionSequence"];
    Graph& actions = actionSequence->graph();
    uint endeff_index = world.getShapeByName("graspRef")->index;
    uint hand_index = world.getShapeByName("eff")->index;

    //-- set up the MotionProblem
    MP.T=2*actions.N*microSteps;
    world.swift().initActivations(world);
    MP.world.watch(false);

    //-- decide on pickAndPlace times
    uintA tPick(actions.N), tPlace(actions.N), idObject(actions.N);
    for(uint i=0;i<actions.N;i++){
      tPick(i) = (2*i+1)*microSteps;
      tPlace(i) = (2*i+2)*microSteps;
      idObject(i) = world.getShapeByName(actions(i)->parents(1)->keys(1))->index;
    }

    //-- transitions
    {
      Task *t;
      t = MP.addTask("transitions", new TransitionTaskMap(world));
      if(microSteps>3) t->map.order=2;
      else t->map.order=1;
      t->setCostSpecs(0, MP.T, {0.}, 1e-1);
    }

    //-- pose
    {
      Task *t;
      t = MP.addTask("pose", new TaskMap_qItself());
      t->map.order=0;
      t->setCostSpecs(0, MP.T, {0.}, 1e-5);
    }

    //-- tasks
    {
      Task *t;
      DefaultTaskMap *m;
      //pick & place position
      t = MP.addTask("pap_pos", m=new DefaultTaskMap(posDiffTMT));
      m->referenceIds.resize(MP.T+1,2) = -1;
      t->prec.resize(MP.T+1).setZero();
      t->target.resize(MP.T+1,3).setZero();
      for(uint i=0;i<actions.N;i++){
        //pick
        m->referenceIds(tPick(i),0) = endeff_index;
        m->referenceIds(tPick(i),1) = idObject(i);
        t->prec(tPick(i))=posPrec;
        //      t->target[tPick(i)]=ARRAY( world_initial.shapes(idObject(i))->X.pos );

        //place
        m->referenceIds(tPlace(i),0) = idObject(i);
        t->prec(tPlace(i))=posPrec;
        t->target[tPlace(i)]=ARRAY( world_final.shapes(idObject(i))->X.pos );
      }

      //pick & place quaternion
      t = MP.addTask("psp_quat", m=new DefaultTaskMap(quatDiffTMT));
      m->referenceIds.resize(MP.T+1,2) = -1;
      t->prec.resize(MP.T+1).setZero();
      t->target.resize(MP.T+1,4).setZero();
      for(uint i=0;i<actions.N;i++){
        //pick
        m->referenceIds(tPick(i),0) = endeff_index;
        m->referenceIds(tPick(i),1) = idObject(i);
        t->prec(tPick(i))=posPrec;
        //      t->target[tPlace(i)]=ARRAY( world_initial.shapes(idObject(i))->X.rot );

        //place
        m->referenceIds(tPlace(i),0) = idObject(i);
        t->prec(tPlace(i))=posPrec;
        t->target[tPlace(i)]=ARRAY( world_final.shapes(idObject(i))->X.rot );
      }

      // zero position velocity
      if(microSteps>3){
        t = MP.addTask("psp_zeroPosVel", m=new DefaultTaskMap(posTMT, endeff_index));
        t->map.order=1;
        t->prec.resize(MP.T+1).setZero();
        for(uint i=0;i<actions.N;i++){
          t->prec(tPick(i))=posPrec;
          t->prec(tPlace(i))=posPrec;
        }

        // zero quaternion velocity
        t = MP.addTask("pap_zeroQuatVel", new DefaultTaskMap(quatTMT, endeff_index));
        t->map.order=1;
        t->prec.resize(MP.T+1).setZero();
        for(uint i=0;i<actions.N;i++){
          t->prec(tPick(i))=posPrec;
          t->prec(tPlace(i))=posPrec;
        }
      }

      // zero grasp joint motion during holding
      ors::Joint *j_grasp = world.getJointByName("graspJoint");
      arr M(j_grasp->qDim(),world.getJointStateDimension());
      M.setZero();
      for(uint i=0;i<j_grasp->qDim();i++) M(i,j_grasp->qIndex+i)=1.;
      cout <<M <<endl;
      t = MP.addTask("graspJoint", new TaskMap_qItself(M));
      t->map.order=1;
      t->prec.resize(MP.T+1).setZero();
      for(uint i=0;i<actions.N;i++){
        for(uint time=tPick(i)+1;time<tPlace(i);time++) t->prec(time)=posPrec;
      }

      // up/down velocities after/before pick/place
      if(microSteps>3){
        t = MP.addTask("pap_upDownPosVel", new DefaultTaskMap(posTMT, endeff_index));
        t->map.order=1;
        t->prec.resize(MP.T+1).setZero();
        t->target.resize(MP.T+1,3).setZero();
        for(uint i=0;i<actions.N;i++){
          t->prec(tPick(i)+2)=posPrec;
          t->target[tPick(i)+2] = {0.,0.,+.1};

          t->prec(tPlace(i)-2)=posPrec;
          t->target[tPlace(i)-2] = {0.,0.,-.1};
        }
      }
    }

    //-- collisions
    {
      Task *t;
      ProxyConstraint *m;

      //of the object itself
      if(microSteps>3){
        t = MP.addTask("object_collisions", m=new ProxyConstraint(allVsListedPTMT, uintA(), margin, true));
        m->proxyCosts.shapes.resize(MP.T+1,1) = -1;
        t->prec.resize(MP.T+1).setZero();
        for(uint i=0;i<actions.N;i++){
          for(uint time=tPick(i)+3;time<tPlace(i)-3;time++){
            m->proxyCosts.shapes(time,0)=idObject(i);
            t->prec(time)=1.;
          }
        }
      }

      //of the hand
      t = MP.addTask("hand_collisions", m=new ProxyConstraint(allVsListedPTMT, uintA(), margin, true));
      m->proxyCosts.shapes.resize(MP.T+1,1) = -1;
      t->prec.resize(MP.T+1).setZero();
      for(uint time=0;time<=MP.T; time++){
        m->proxyCosts.shapes(time,0)=hand_index;
        t->prec(time)=1.;
      }
    }

    //-- graph switches
    for(uint i=0;i<actions.N;i++){
      //pick at time 2*i+1
      ors::KinematicSwitch *op_pick = new ors::KinematicSwitch();
      op_pick->symbol = ors::KinematicSwitch::addRigid;
      op_pick->timeOfApplication = tPick(i)+1;
      op_pick->fromId = world.shapes(endeff_index)->index;
      op_pick->toId = world.shapes(idObject(i))->index;
      MP.switches.append(op_pick);

      //place at time 2*i+2
      ors::KinematicSwitch *op_place = new ors::KinematicSwitch();
      op_place->symbol = ors::KinematicSwitch::deleteJoint;
      op_place->timeOfApplication = tPlace(i)+1;
      op_place->fromId = world.shapes(endeff_index)->index;
      op_place->toId = world.shapes(idObject(i))->index;
      MP.switches.append(op_place);
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

double optimSwitchConfigurations(ors::KinematicWorld& world_initial, ors::KinematicWorld& world_final,
                                 Graph& symbolicState,
                                 uint microSteps){
  SwitchConfigurationProgram f(world_initial, world_final, symbolicState, microSteps, 0);

  arr x = replicate(f.MP.x0, f.MP.T+1); //we initialize with a constant trajectory!
//  rndGauss(x,.01,true); //don't initialize at a singular config

  OptConstrained opt(x, NoArr, f, OPT(verbose=2, damping = 1e-1, stopTolerance=1e-2, maxStep=.5));
  opt.run();
  f.MP.costReport();
  for(;;) displayTrajectory(x, 1, f.MP.world, f.MP.switches, "planned configs", .02);
  return opt.UCP.get_sumOfSquares();
}

//===========================================================================

