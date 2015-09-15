#include "RelationalMachineModule.h"

struct RM_EditCallback:GraphEditCallback{
  RelationalMachineModule &RMM;
  Log& _log;
  RM_EditCallback(RelationalMachineModule &RMM):RMM(RMM), _log(RMM._log){}
  virtual void cb_new(Node *it){
    LOG(3) <<"state cb -- new fact: " <<*it;
  }
  virtual void cb_delete(Node *it){
    LOG(3) <<"state cb -- del fact: " <<*it;
    bool lockedHere=false;
    if(!RMM.A.v->rwlock.isLocked()){ lockedHere=true; RMM.A.writeAccess(); }
    for(Activity *act:RMM.A()) if(act->fact==it){
      LOG(3) <<"removing activity '" <<*act <<"'";
      RMM.A().removeValue(act);
      delete act;
    }
    if(lockedHere) RMM.A.deAccess();
  }
};

RelationalMachineModule::RelationalMachineModule():Module("RelationalMachineModule"),
  _log("RelationalMachineModule"){
}

RelationalMachineModule::~RelationalMachineModule(){
}

void RelationalMachineModule::open(){
  RM.writeAccess();
  RM().init("machine.fol");
  RM().state->callbacks.append(new RM_EditCallback(*this));
  RM.deAccess();
  threadStep(1);
}

void RelationalMachineModule::close(){
  listDelete(RM.set()->state->callbacks);
}

void RelationalMachineModule::step(){
  effects.writeAccess();
  MT::String effs = effects();
  effects().clear();
  effects.deAccess();
  LOG(1) <<std::setprecision(2) <<std::fixed <<MT::realTime() <<"sec: it=" <<RM.var->revision.getValue()<<" EFFECT=" <<effs;
  if(!effs.N && step_count) return; //on 1st iteration we need a step!

  RM.writeAccess();
  A.writeAccess();
  if(effs.N){
    RM().applyEffect(effs);
    RM().fwdChainRules();
    LOG(2) <<"STATE =\n  " <<RM().getState();
    LOG(4) <<"KB =\n  " <<RM().getKB();
  }

  if(!step_count || effs.N){
    state.set() = RM().getState();

    //-- sync with activities: add activities for non-associated
    Graph &state = *RM().state;
    MT::Array<Activity*> fact2act(state.N);
    fact2act.setUni(NULL);
    LOG(3) <<"Syncing facts with activities..";
    for(Activity *act:A()){
      CHECK(act->fact == state(act->fact->index),"SOMETHING'S WRONG!");
      fact2act(act->fact->index) = act;
    }
    for(Node *it:state) if(fact2act(it->index)==NULL){ //fact is not associated yet
      Activity *act = newActivity(it);
      if(act){
        A().append(act);
        LOG(3) <<"added activity '" <<*act <<"' for fact '" <<*it <<"'";
      }else{
        LOG(3) <<"Fact '" <<*it <<"' cannot be matched/created with an activity";
      }
    }
  }
  A.deAccess();
  RM.deAccess();

  //TODO: cleanup? remove NULL facts from state?
}

