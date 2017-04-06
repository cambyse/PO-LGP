/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "pathProblem.h"
#include <Kin/taskMaps.h>
#include <Kin/kin_swift.h>
#include <Optim/lagrangian.h>

//===========================================================================

PathProblem::PathProblem(const mlr::KinematicWorld& world_initial,
                         const mlr::KinematicWorld& world_final,
                         const Graph& symbolicState,
                         uint microSteps,
                         int verbose)
  : world(world_initial), symbolicState(symbolicState), microSteps(microSteps), verbose(verbose), MP(world){
  //  ConstrainedProblem::operator=( conv_KOrderMarkovFunction2ConstrainedProblem(MP.komo_problem) );

  double posPrec = mlr::getParameter<double>("LGP/precision", 1e3);
//  double colPrec = mlr::getParameter<double>("LGP/collisionPrecision", -1e0);
  double margin = mlr::getParameter<double>("LGP/collisionMargin", .05);

  //get the actions!
  Node *actionSequence=symbolicState["actionSequence"];
  Graph& actions = actionSequence->graph();
  uint endeff_index = world.getShapeByName("graspRef")->index;
  uint hand_index = world.getShapeByName("eff")->index;

  //-- set up the MotionProblem
  MP.T=2*actions.N*microSteps;
  world.swift().initActivations(world);
//  MP.world.watch(false);

  //-- decide on pickAndPlace times
  uintA tPick(actions.N), tPlace(actions.N), idObject(actions.N);
  for(uint i=0;i<actions.N;i++){
    tPick(i) = (2*i+1)*microSteps;
    tPlace(i) = (2*i+2)*microSteps;
    idObject(i) = world.getShapeByName(actions(i)->parents(1)->keys.last())->index;
  }

  //-- transitions
  {
    Task *t;
    t = MP.addTask("transitions", new TaskMap_Transition(world), OT_sumOfSqr);
    if(microSteps>3) t->map->order=2;
    else t->map->order=1;
    t->setCostSpecs(0, MP.T, {0.}, 1e-1);
  }

  //-- pose damping
  {
    Task *t;
    t = MP.addTask("pose", new TaskMap_qItself(), OT_sumOfSqr);
    t->map->order=0;
    t->setCostSpecs(0, MP.T, {0.}, 1e-5);
  }

  //-- tasks
  {
    Task *t;
    TaskMap_Default *m;
    //pick & place position
    t = MP.addTask("pap_pos", m=new TaskMap_Default(posDiffTMT), OT_sumOfSqr);
    m->referenceIds.resize(MP.T+1,2) = -1;
    t->prec.resize(MP.T+1).setZero();
    t->target.resize(MP.T+1,3).setZero();
    for(uint i=0;i<actions.N;i++){
      //pick
      m->referenceIds(tPick(i),0) = endeff_index;
      m->referenceIds(tPick(i),1) = idObject(i);
      t->prec(tPick(i))=posPrec;
      //      t->target[tPick(i)]=conv_vec2arr( world_initial.shapes(idObject(i))->X.pos );

      //place
      m->referenceIds(tPlace(i),0) = idObject(i);
      t->prec(tPlace(i))=posPrec;
      t->target[tPlace(i)]=conv_vec2arr( world_final.shapes(idObject(i))->X.pos );
    }

    //pick & place quaternion
    t = MP.addTask("psp_quat", m=new TaskMap_Default(quatDiffTMT), OT_sumOfSqr);
    m->referenceIds.resize(MP.T+1,2) = -1;
    t->prec.resize(MP.T+1).setZero();
    t->target.resize(MP.T+1,4).setZero();
    for(uint i=0;i<actions.N;i++){
      //pick
      m->referenceIds(tPick(i),0) = endeff_index;
      m->referenceIds(tPick(i),1) = idObject(i);
      t->prec(tPick(i))=posPrec;
      //      t->target[tPlace(i)]=conv_quat2arr( world_initial.shapes(idObject(i))->X.rot );

      //place
      m->referenceIds(tPlace(i),0) = idObject(i);
      t->prec(tPlace(i))=posPrec;
      t->target[tPlace(i)]=conv_quat2arr( world_final.shapes(idObject(i))->X.rot );
    }

    // zero position velocity
    if(microSteps>3){
      t = MP.addTask("psp_zeroPosVel", m=new TaskMap_Default(posTMT, endeff_index), OT_sumOfSqr);
      t->map->order=1;
      t->prec.resize(MP.T+1).setZero();
      for(uint i=0;i<actions.N;i++){
        t->prec(tPick(i))=posPrec;
        t->prec(tPlace(i))=posPrec;
      }

      // zero quaternion velocity
      t = MP.addTask("pap_zeroQuatVel", new TaskMap_Default(quatTMT, endeff_index), OT_sumOfSqr);
      t->map->order=1;
      t->prec.resize(MP.T+1).setZero();
      for(uint i=0;i<actions.N;i++){
        t->prec(tPick(i))=posPrec;
        t->prec(tPlace(i))=posPrec;
      }
    }

    // zero grasp joint motion during holding
//    mlr::Joint *j_grasp = world.getJointByName("graspJoint");
//    arr M(j_grasp->qDim(),world.getJointStateDimension());
//    M.setZero();
//    for(uint i=0;i<j_grasp->qDim();i++) M(i,j_grasp->qIndex+i)=1.;
//    cout <<M <<endl;
    t = MP.addTask("graspJoint", new TaskMap_qItself(QIP_byJointNames, {"graspJoint"}, world), OT_sumOfSqr);
    t->map->order=1;
    t->prec.resize(MP.T+1).setZero();
    for(uint i=0;i<actions.N;i++){
      for(uint time=tPick(i)+1;time<tPlace(i);time++) t->prec(time)=posPrec;
    }

    // up/down velocities after/before pick/place
    if(microSteps>3){
      t = MP.addTask("pap_upDownPosVel", new TaskMap_Default(posTMT, endeff_index), OT_sumOfSqr);
      t->map->order=1;
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
      t = MP.addTask("object_collisions", m=new ProxyConstraint(allVsListedPTMT, uintA(), margin, true), OT_ineq);
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
    t = MP.addTask("hand_collisions", m=new ProxyConstraint(allVsListedPTMT, uintA(), margin, true), OT_ineq);
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
    mlr::KinematicSwitch *op_pick = new mlr::KinematicSwitch();
    op_pick->symbol = mlr::KinematicSwitch::addJointZero;
    op_pick->jointType = mlr::JT_rigid;
    op_pick->timeOfApplication = tPick(i)+1;
    op_pick->fromId = world.shapes(endeff_index)->index;
    op_pick->toId = world.shapes(idObject(i))->index;
    MP.switches.append(op_pick);

    //place at time 2*i+2
    mlr::KinematicSwitch *op_place = new mlr::KinematicSwitch();
    op_place->symbol = mlr::KinematicSwitch::deleteJoint;
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
      t = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {0}, margin));
      t->setCostSpecs(0, MP.T, {0.}, colPrec);
    }
*/

}

//===========================================================================

double PathProblem::optimize(arr& x){
  x = MP.getInitialization();
//  rndGauss(x,.01,true); //don't initialize at a singular config

  Conv_KOMO_ConstrainedProblem CP(MP.komo_problem);
  OptConstrained opt(x, NoArr, CP, OPT(verbose=2, damping = 1e-1, stopTolerance=1e-2, maxStep=.5));
  opt.run();
  MP.costReport();
//  for(;;)
    displayTrajectory(x, 1, MP.world, MP.switches, "planned configs", .02);
  return opt.UCP.get_costs();
}

//===========================================================================

