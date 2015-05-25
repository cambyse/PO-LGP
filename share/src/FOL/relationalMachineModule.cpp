#include "relationalMachineModule.h"

RelationalMachineModule::RelationalMachineModule():Module("RelationalMachineModule"){
}

RelationalMachineModule::~RelationalMachineModule(){
}

void RelationalMachineModule::open(){
  MT::open(fil,"z.RelationalMachineModule");
  RM.set()->init("machine.fol");
  RM.set()->verbose = false;
  threadStep(1);
}

void RelationalMachineModule::step(){
  //-- step all activities
  A.readAccess();
  for(Activity *act:A()) act->step(0.01);
  A.deAccess();

  effects.writeAccess();
  MT::String effs = effects();
  effects().clear();
  effects.deAccess();
  if(!effs.N) return;

  RM.writeAccess();
  if(effs.N){
    RM().applyEffect(effs);
    RM().fwdChainRules();
    fil <<RM.var->revision.getValue() <<endl <<effs <<endl <<RM().getState() <<endl <<RM().getKB() << endl;
  }

  if(!step_count || effs.N){
    state.set() = RM().getState();

    //-- sync with activities
    Graph &state = *RM().state;
    MT::Array<Activity*> it2act(state.N);
    if(state.N){
      it2act = NULL;
      LOG(2) <<"Syncing facts with activities..";
      for(Activity *act:A()){
        uint idx = act->fact->index;
        if(idx>=state.N || state(idx)!=act->fact){
          act->fact=NULL;
          LOG(2) <<"  Activity '" <<*act <<"' has no fact match";
        }else{
          it2act(idx) = act;
          LOG(2) <<"  Activity '" <<*act <<"' matches fact '" <<&act->fact <<"'";
        }
      }
    }

    A.writeAccess();
    //-- del NULL activities
    for(uint i=A().N; i--;){
      Activity *act =A()(i);
      if(act->fact==NULL){
        LOG(2) <<"removing activity '" <<*act <<"'";
        A().removeValue(act);
        delete act;
      }
    }

    //-- add activities for non-associated
    for(Item *it:state){
      if(it2act(it->index)==NULL){
        Activity *act = newActivity(it);
        if(act){
          A().append(act);
          LOG(2) <<"added activity '" <<*act <<"' for fact '" <<*it <<"'";
        }else{
          LOG(2) <<"Fact '" <<*it <<"' cannot be matched/created with an activity";
        }
      }else{
        LOG(2) <<"Fact '" <<*it <<"' matches activity '" <<*it2act(it->index) <<"'";
      }
    }
    A.deAccess();
  }
  RM.deAccess();

  //TODO: cleanup? remove NULL facts from state?
}

void RelationalMachineModule::close(){
  fil.close();
}
