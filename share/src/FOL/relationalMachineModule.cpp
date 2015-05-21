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
  for(Activity *act:A()) act->step(RM(), 0.01);
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
    //-- sync with activities
    Graph &state = *RM().state;
    MT::Array<Activity*> it2act(state.N);
    it2act = NULL;
    for(Activity *act:A()){
      uint idx = act->fact->index;
      CHECK(state(idx)==act->fact,"");
      it2act(idx) = act;
    }

    A.writeAccess();
    //-- del NULL activities
    for(Activity *act:A()) if(act->fact->getValueDirectly()==NULL){
      A().removeValue(act);
      delete act;
    }

    //-- add activities for non-associated
    for(Item *it:state){
      if(it2act(it->index)==NULL){
        Activity *act = newActivity(it);
        if(act) A().append(act);
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
