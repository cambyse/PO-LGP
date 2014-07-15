#include "actions.h"
#include "actionMachine_internal.h"

//===========================================================================
Singleton<SymbolL> symbols;

//===========================================================================
// ActionMachine
//
ActionMachine::ActionMachine():Module("ActionMachine"){
  ActionL::memMove=true;
  s = new sActionMachine();
}

ActionMachine::~ActionMachine(){
  delete s;
}

void ActionMachine::open(){
  s->world.getJointState(s->q, s->qdot);

  s->MP.H_rate_diag = pr2_reasonable_W(s->world);
  s->MP.qitselfPD.y_ref = s->q;
//  s->MP.qitselfPD.setGains(1.,10.);

  //-- wait for first q observation!
  if(MT::getParameter<bool>("useRos",false)){
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      ctrl_obs.var->waitForNextRevision();
      if(ctrl_obs.get()->q.N==s->MP.world.q.N
         && ctrl_obs.get()->qdot.N==s->MP.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    s->q = ctrl_obs.get()->q;
    s->qdot = ctrl_obs.get()->qdot;
    s->MP.setState(s->q, s->qdot);
  }
  s->zero_qdot.resize(s->qdot.N).setZero();
  //arr fL_base = S.fL_obs.get();
}

void ActionMachine::step(){
  static uint t=0;
  t++;

  if(!(t%10))
    s->MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

  transition();

  //defaults
  s->refs.fR = ARR(0., 0., 0.);
  s->refs.fR_gainFactor = 0.;
  s->refs.Kp_gainFactor = 1.;

  arr joypadState = joystickState.get();
  if(stopButtons(joypadState)) engine().shutdown.incrementValue();

  //-- get access
  A.readAccess();

  //  cout <<"** active actions:";
  reportActions(A());

  for(GroundedAction *a : A()) {
    if(a->actionState==ActionState::active){
      cout << a->name << ": " << a->ID << endl;
      for(PDtask *t:a->tasks) t->active=true;

      if(a->name == "PushForce") {
        cout <<" - FORCE TASK: " << endl;
        PushForce* pf = dynamic_cast<PushForce*>(a);
        // cout << pf->forceVec << endl;
        s->refs.fR = pf->forceVec;
        s->refs.fR_gainFactor = 1.;
        s->refs.Kp_gainFactor = .2;
      }
    }
    else {
      for(PDtask *t:a->tasks) t->active=false;
    }
  }

  cout <<"FL=" <<ctrl_obs.get()->fL <<endl;

  for(uint tt=0;tt<10;tt++){
    arr a = s->MP.operationalSpaceControl();
    s->q += .001*s->qdot;
    s->qdot += .001*a;
    s->MP.setState(s->q, s->qdot);
  }

  A.deAccess();

  s->refs.q=s->q;
  s->refs.qdot=s->zero_qdot;
  ctrl_ref.set() = s->refs;
}

void ActionMachine::close(){
}

GroundedAction* ActionMachine::add(GroundedAction *action,
                                   ActionState actionState)
{
  action->initYourself(*this);
  action->actionState = actionState;
  A.set()->append(action);
  return action;
}

void ActionMachine::add_sequence(GroundedAction *action1,
                                 GroundedAction *action2,
                                 GroundedAction *action3,
                                 GroundedAction *action4)
{
  this->add(action1);
  this->add(action2, ActionState::queued);
  action2->dependsOnCompletion.append(action1);
  if (action3) {
    this->add(action3, ActionState::queued);
    action3->dependsOnCompletion.append(action2);
  }
  if (action4) {
    this->add(action4, ActionState::queued);
    action4->dependsOnCompletion.append(action3);
  }
}

void ActionMachine::removeGroundedAction(GroundedAction* a, bool hasLock){
  a->deinitYourself(*this);
  if(!hasLock) A.set()->removeValue(a);
  else A().removeValue(a);
}

void ActionMachine::transition(){
  Access_typed<ActionL>::WriteToken lock(&A); //kills itself
  //const auto& lock=A.set();

  //-- first remove all old successes and fails
  for_list_rev(GroundedAction, a, A()) if(a->actionState==ActionState::success || a->actionState==ActionState::failed){
    removeGroundedAction(a, true);
    a=NULL; //a has deleted itself, for_list_rev should be save, using a_COUNTER
  }

  //-- check new successes and fails
  for(GroundedAction *a:A()) if(a->actionState==ActionState::active){
    if(a->finishedSuccess(*this)) a->actionState=ActionState::success;
    if(a->finishedFail(*this)) a->actionState=ActionState::failed;
  }

  //-- progress with queued
  for(GroundedAction *a:A()) if(a->actionState==ActionState::queued){
    bool fail=false, succ=true;
    for(GroundedAction *b:a->dependsOnCompletion){
      if(b->actionState==ActionState::failed) fail=true;
      if(b->actionState!=ActionState::success) succ=false;
    }
    if(fail) a->actionState=ActionState::failed; //if ONE dependence failed -> fail
    if(succ) a->actionState=ActionState::active; //if ALL dependences succ -> active
    //in all other cases -> queued
  }
}

void ActionMachine::waitForActionCompletion(GroundedAction* a){
  for(bool cont=true;cont;){
    A.var->waitForNextRevision();
    A.readAccess();
    if(!A().contains(a)) cont=false;
    A.deAccess();
  }
}

void ActionMachine::waitForActionCompletion() {
  bool cont = true;
  while (cont) {
    A.var->waitForNextRevision();
    A.readAccess();
    if (A().N == 0 || (A().N == 1 && A()(0)->name == "CoreTasks")) {
      cont=false;
    }
    A.deAccess();
  }
}
//===========================================================================
// GroundedAction
//
const char* GroundedAction::GroundActionValueString[7] = {
 "trueLV", "falseLV", "inactive", "queued", "active", "failed", "success"
};

void GroundedAction::deinitYourself(ActionMachine& actionMachine) {
  for (PDtask *t : tasks) actionMachine.s->MP.tasks.removeValue(t);
  listDelete(tasks);
}

//===========================================================================
// Helper functions
//
void reportExistingSymbols(){
  for(Symbol *s:symbols()){
    cout <<"Symbol '" <<s->name <<"' nargs=" <<s->nargs;
    cout <<endl;
  }
}

void reportActions(ActionL &A){
  cout <<"* ActionL" <<endl;
  for (GroundedAction *a : A){
    cout << a->name << " actionState=" << a->getActionStateString() << endl;
    // cout <<a->symbol.name <<" {" <<a->shapeArg1 <<' ' <<a->shapeArg2 <<" } { " <<a->poseArg1 <<' ' <<a->poseArg2 <<" }:";
  }
}
