#include "RelationalMachineModule.h"
#include <iomanip>

struct RM_EditCallback:GraphEditCallback{
  RelationalMachineModule &RMM;
  mlr::LogObject& _log;
  RM_EditCallback(RelationalMachineModule &RMM):RMM(RMM), _log(RMM._log){}
  virtual void cb_new(Node *it){
    LOG(3) <<"state cb -- new fact: " <<*it;
    RMM.threadStep();
  }
  virtual void cb_delete(Node *it){
    LOG(3) <<"state cb -- del fact: " <<*it;
    RMM.A.writeAccess();
    for(Activity *act:RMM.A()) if(act->fact==it){
      LOG(3) <<"removing activity '" <<*act <<"'";
      RMM.A().removeValue(act);
      delete act;
    }
    RMM.A.deAccess();
    RMM.threadStep();
  }
  virtual void cb_edit(Node *it){
    LOG(3) <<"state cb -- edit fact: " <<*it;
    RMM.threadStep();
  }
};

RelationalMachineModule::RelationalMachineModule()
  : Thread("RelationalMachineModule"),
    _log("RelationalMachineModule", 1, 1){
}

RelationalMachineModule::~RelationalMachineModule(){
}

void RelationalMachineModule::open(){
  RM.writeAccess();
  RM().KB <<FILE(mlr::mlrPath("data/keywords.g"));
  RM().init("machine.fol");
  RM().state->callbacks.append(new RM_EditCallback(*this));
  RM.deAccess();
  state.set() = RM.get()->getState();
  symbols.set() = RM.get()->getSymbols();
  threadStep(1);
}

void RelationalMachineModule::close(){
  listDelete(RM.set()->state->callbacks);
}

void RelationalMachineModule::step(){
  effects.writeAccess();
  mlr::String effs = effects();
  effects().clear();
  effects.deAccess();

  LOG(1) <<std::setprecision(2) <<std::fixed <<mlr::realTime() <<"sec: it=" <<RM.data->revLock.revision.getStatus()<<" EFFECT=" <<effs;

  RM.writeAccess();
  if(effs.N) RM().applyEffect(effs);
  RM().fwdChainRules();
  LOG(2) <<"STATE =\n  " <<RM().getState();
  LOG(4) <<"KB =\n  " <<RM().getKB();
  RM.deAccess();

  state.set() = RM.get()->getState();

  if(true || !step_count || effs.N){
    RM.readAccess();
    A.writeAccess();

    //-- sync with activities: add activities for non-associated
    const Graph &RMstate = *RM().state;
    mlr::Array<Activity*> fact2act(RMstate.N);
    fact2act.setZero();
    LOG(3) <<"Syncing facts with activities..";
    for(Activity *act:A()){ //every activity has a pointer act->fact to its fact
      CHECK(act->fact == RMstate(act->fact->index),"SOMETHING'S WRONG!");
      fact2act(act->fact->index) = act;
    }
    for(Node *it:RMstate) if(fact2act(it->index)==NULL){ //fact is not associated yet
      Activity *act = newActivity(it);
      if(act){
        A().append(act);
        LOG(3) <<"added activity '" <<*act <<"' for fact '" <<*it <<"'";
      }else{
        LOG(3) <<"Fact '" <<*it <<"' cannot be matched/created with an activity";
      }
    }
    A.deAccess();
    RM.deAccess();
  }
}

//========================================================================================

void RelationalMachineModule::newSymbol(const char* symbol){
  RM.set()->declareNewSymbol(symbol);
}

void RelationalMachineModule::setFact(const char* fact){
  effects.set()() <<fact <<", ";
  threadStep();
  state.waitForNextRevision(); //TODO: is this robust?
}

void RelationalMachineModule::waitForCondition(const char* query){
  for(;;){
    if(RM.get()->queryCondition(query)) return;
    if(stopWaiting.getStatus()>0) return;
    state.waitForNextRevision();
  }
}

void RelationalMachineModule::runScript(const char* filename){
  FILE(filename) >>RM.set()->KB;

  Graph& script = RM.get()->KB.getNode("Script")->graph();
  int rev=0;
  for(Node* n:script){
    if(n->parents.N==0 && n->isGraph()){ //interpret as wait
      for(;;){
        if(allFactsHaveEqualsInKB(*RM.get()->state, n->graph())) break;
        rev=RM.waitForRevisionGreaterThan(rev);
      }
    }else{ //interpret as set fact
      RM.set()->applyEffect(n, true);
//      S->rmm->threadStep();
//      S->effects.set()() <<"(go)"; //just trigger that the RM module steps
    }
  }
}
