#include "relationalMachineModule.h"

RelationalMachineModule::RelationalMachineModule():Module("RelationalMachineModule"){
}

RelationalMachineModule::~RelationalMachineModule(){
}

void RelationalMachineModule::open(){
  MT::open(fil,"z.RelationalMachineModule");
  RM.set()->init("machine.fol");
  RM.set()->verbose = true;
  threadStep(1);
}

void RelationalMachineModule::step(){
  RM.writeAccess();

  //-- step all activities
  A.readAccess();
  for(Activity *act:A()) act->step(0.01);
  A.deAccess();

  effects.writeAccess();
  MT::String effs = effects();
  effects().clear();
  effects.deAccess();

  if(effs.N){
    RM().applyEffect(effs);
    RM().fwdChainRules();
  }

  if(!step_count || effs.N){
    state.set() = RM().getState();

    //-- sync with activities
    Graph &state = *RM().state;
    MT::Array<Activity*> it2act(state.N);
    if(state.N){
      it2act = NULL;
      cout <<"MATCHES" <<endl;
      for(Activity *act:A()){
        uint idx = act->fact->index;
        if(idx>=state.N || state(idx)!=act->fact){
          act->fact=NULL;
          cout <<"Activity '" <<*act <<"' has no fact match" <<endl;
        }else{
          it2act(idx) = act;
          cout <<"Activity '" <<*act <<"' matches fact '" <<&act->fact <<"'" <<endl;
        }
      }
    }

    A.writeAccess();
    //-- del NULL activities
    for(uint i=A().N; i--;){
      Activity *act =A()(i);
      if(act->fact==NULL){
        LOG(-1) <<"removing activity '" <<*act <<"'";
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
          cout <<"added activity '" <<*act <<"' for fact '" <<*it <<"'" <<endl;
        }else{
          cout <<"Fact '" <<*it <<"' cannot be matched/created with an activity" <<endl;
        }
      }else{
        cout <<"Fact '" <<*it <<"' matches activity '" <<*it2act(it->index) <<"'" <<endl;
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
