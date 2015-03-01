#include "actionMachine_internal.h"

#include <Hardware/gamepad/gamepad.h>
#include <Motion/pr2_heuristics.h>
#include <FOL/fol.h>

//===========================================================================
//Singleton<SymbolL> symbols;

const char* ActionStateString[7] = {
 "trueLV", "falseLV", "inactive", "queued", "active", "failed", "success"
};

const char* getActionStateString(ActionState actionState){ return ActionStateString[actionState]; }

//===========================================================================
//
// ActionMachine
//

ActionMachine::ActionMachine():Module("ActionMachine"){
  ActionL::memMove=true;
  Kq_gainFactor = ARR(1.);
  Kd_gainFactor = ARR(1.);
  s = new sActionMachine();
  world = &s->world;
}

ActionMachine::~ActionMachine(){
  delete s;
}

void ActionMachine::open(){
  s->world.getJointState(s->q, s->qdot);

  s->feedbackController.H_rate_diag = MT::getParameter<double>("Hrate", 1.)*pr2_reasonable_W(s->world);
  s->feedbackController.qitselfPD.y_ref = s->q;
  s->feedbackController.qitselfPD.setGains(.0,10.);

  KB.writeAccess();
  FILE("machine.fol") >> KB();
  KB().checkConsistency();
  KB()>> FILE("z.initialKB");
  KB.deAccess();

  KB.readAccess();
  Item *tasks = KB()["Tasks"];
  if(tasks){
    parseTaskDescriptions(tasks->kvg());
  }
  KB.deAccess();

  MT::open(fil,"z.actionMachine");

  bool useRos = MT::getParameter<bool>("useRos",false);
  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    uint trials=0;
    for(;useRos;){
      ctrl_obs.var->waitForNextRevision();
      cout <<"REMOTE joint dimension=" <<ctrl_obs.get()->q.N <<endl;
      cout <<"LOCAL  joint dimension=" <<s->feedbackController.world.q.N <<endl;

      if(ctrl_obs.get()->q.N==s->feedbackController.world.q.N
         && ctrl_obs.get()->qdot.N==s->feedbackController.world.q.N)
        break;

      trials++;
      if(trials>20){
        HALT("sync'ing real PR2 with simulated failed - using useRos=false")
      }
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    s->q = ctrl_obs.get()->q;
    s->qdot = ctrl_obs.get()->qdot;
    s->feedbackController.setState(s->q, s->qdot);
  }
}

void ActionMachine::step(){
  static uint t=0;
  t++;

  if(!(t%20))
    s->feedbackController.world.watch(false, STRING("local operational space controller state t="<<(double)t/100.));

  //-- do the logic of transitioning between actions, stopping/sequencing them, querying their state
//  transition();
  transitionFOL( .01*t,  (t<=1) );

  //-- set gains to default value (can be overwritten by other actions)
  Kq_gainFactor=ARR(1.);
  Kd_gainFactor=ARR(1.);

  //-- check the gamepad
  arr gamepad = gamepadState.get();
  if(stopButtons(gamepad)) engine().shutdown.incrementValue();

  //-- get access to the list of actions
  A.writeAccess();

  //  cout <<"** active actions:";
//  reportActions(A());

  //-- code to output force signals
  if(true){
    ors::Shape *ftL_shape = world->getShapeByName("endeffForceL");
    arr fLobs = ctrl_obs.get()->fL;
    arr uobs =  ctrl_obs.get()->u_bias;
//    cout <<fLobs <<endl;
    if(fLobs.N && uobs.N){
      arr Jft, J;
      world->kinematicsPos(NoArr,J,ftL_shape->body,&ftL_shape->rel.pos);
      world->kinematicsPos_wrtFrame(NoArr,Jft,ftL_shape->body,&ftL_shape->rel.pos,world->getShapeByName("l_ft_sensor"));
      Jft = inverse_SymPosDef(Jft*~Jft)*Jft;
      J = inverse_SymPosDef(J*~J)*J;
      MT::arrayBrackets="  ";
      fil <<t <<' ' <<zeros(3) <<' ' << Jft*fLobs << " " << J*uobs << endl;
      MT::arrayBrackets="[]";
    }
  }

  //-- call the step method for each action
  for(Action *a : A()) {
    if(a->actionState==ActionState::active){
      a->step(*this);
      for(CtrlTask *t:a->tasks) t->active=true;
      a->actionTime += .01;
    } else {
      for(CtrlTask *t:a->tasks) t->active=false;
    }
  }

  //-- compute the feedback controller step
  //first collect all tasks of all actions into the feedback controller:
  s->feedbackController.tasks.clear();
  for(Action *a : A()) for(CtrlTask *t:a->tasks) s->feedbackController.tasks.append(t);
  //now operational space control
  for(uint tt=0;tt<10;tt++){
    arr a = s->feedbackController.operationalSpaceControl();
    s->q += .001*s->qdot;
    s->qdot += .001*a;
    s->feedbackController.setState(s->q, s->qdot);
  }

  // s->MP.reportCurrentState();
  A.deAccess();

  //-- send the computed movement to the robot
  s->refs.fR = ARR(0., 0., 0.);
  s->refs.Kq_gainFactor = Kq_gainFactor;
  s->refs.Kd_gainFactor = Kd_gainFactor;

  s->refs.q=s->q;
  s->refs.qdot = zeros(s->q.N);
  s->refs.u_bias = zeros(s->q.N);
  s->refs.gamma = 1.;
  ctrl_ref.set() = s->refs;
}

void ActionMachine::close(){
  fil.close();
}

void ActionMachine::parseTaskDescriptions(const KeyValueGraph& T){
  cout <<"Instantiating task descriptions:\n" <<T <<endl;
  for(Item *t:T){
    Graph &td = t->kvg();
    MT::String type=td["type"]->V<MT::String>();
    if(type=="homing"){
      new Homing(*this, t->parents(0)->keys(1));
    }else{
      DefaultTaskMap *map = new DefaultTaskMap(td, *world);
      CtrlTask* task = new CtrlTask(t->parents(0)->keys(1), *map, td);
      new FollowReference(*this, t->parents(0)->keys(1), task);
    }
  }
}

void ActionMachine::add_sequence(Action *action1,
                                 Action *action2,
                                 Action *action3,
                                 Action *action4)
{
  action2->actionState = ActionState::queued;
  action2->dependsOnCompletion.append(action1);
  if (action3) {
    action3->actionState = ActionState::queued;
    action3->dependsOnCompletion.append(action2);
  }
  if (action4) {
    action4->actionState = ActionState::queued;
    action4->dependsOnCompletion.append(action3);
  }
}

void ActionMachine::removeAction(Action* a, bool hasLock){
  if(!hasLock) A.set()->removeValue(a);
  else A().removeValue(a);
  delete a;
}

void ActionMachine::transition(){
  A.writeAccess();

  //-- first remove all old successes and fails
  for_list_rev(Action, a, A()) if(a->actionState==ActionState::success || a->actionState==ActionState::failed){
    removeAction(a, true);
    a=NULL; //a has deleted itself, for_list_rev should be save, using a_COUNTER
  }

  //-- check new successes and fails
  for(Action *a:A()) if(a->actionState==ActionState::active){
    if(a->finishedSuccess(*this)) a->actionState=ActionState::success;
    if(a->finishedFail(*this)) a->actionState=ActionState::failed;
  }

  //-- progress with queued
  for(Action *a:A()) if(a->actionState==ActionState::queued){
    bool fail=false, succ=true;
    for(Action *b:a->dependsOnCompletion){
      if(b->actionState==ActionState::failed) fail=true;
      if(b->actionState!=ActionState::success) succ=false;
    }
    if(fail) a->actionState=ActionState::failed; //if ONE dependence failed -> fail
    if(succ) a->actionState=ActionState::active; //if ALL dependences succ -> active
    //in all other cases -> queued
  }

  A.deAccess();
}

void ActionMachine::transitionFOL(double time, bool forceChaining){
  bool changes=false;
  KB.writeAccess();
  //-- check new successes and fails and add to symbolic state
  Item* convSymbol = KB().getItem("conv");
  Item* timeoutSymbol = KB().getItem("timeout");
  A.readAccess();
  for(Action *a:A()) if(a->actionState==ActionState::active){
    if(a->finishedSuccess(*this)){
      Item *newit = KB.data()->append<bool>(STRINGS_0(), {a->symbol, convSymbol}, new bool(true), true);
      if(getEqualFactInKB(KB(), newit)) delete newit;
      else changes=true;
    }
  }
  A.deAccess();

  if(changes || forceChaining){
    ItemL state = getLiteralsOfScope(KB());
    cout <<"STATE (changed by real world at t=" <<time <<"):"; listWrite(state, cout); cout <<endl;
    forwardChaining_FOL(KB(), NULL, false);
    state = getLiteralsOfScope(KB());
    cout <<"STATE (transitioned by FOL   at t=" <<time <<"):"; listWrite(state, cout); cout <<endl;

    A.writeAccess();
    for(Action *a:A()){
      bool act=false;
      for(Item *lit:a->symbol->parentOf){
        if(&lit->container==&KB() && lit->parents.N==1){ act=true; break; }
      }
      if(act) a->actionState=ActionState::active;
      else a->actionState=ActionState::queued;
    }
    A.deAccess();
  }
  KB.deAccess();
}

void ActionMachine::waitForActionCompletion(Action* a){
  for(bool cont=true;cont;){
    A.waitForNextRevision();
    A.readAccess();
    if(!A().contains(a)) cont=false;
    A.deAccess();
  }
}

void ActionMachine::waitForActionCompletion() {
  bool cont = true;
  while (cont) {
    A.waitForNextRevision();
    A.readAccess();
    if (A().N == 0 || (A().N == 1 && A()(0)->name == "CoreTasks")) {
      cont=false;
    }
    A.deAccess();
  }
}

void ActionMachine::waitForQuitSymbol() {
  bool cont = true;
  while (cont) {
    KB.waitForNextRevision();
    KB.readAccess();
    Item* quitSymbol = KB().getItem("quit");
    for(Item *f:quitSymbol->parentOf){
      if(&f->container==&KB() && f->parents.N==1){ cont=false; break; }
    }
    KB.deAccess();
  }
}

//===========================================================================
// Helper functions
//

void reportActions(ActionL &A){
  cout <<"** REPORT ON CURRENT STATE OF ACTIONS" <<endl;
  for (Action *a : A) a->reportState(cout);
}
