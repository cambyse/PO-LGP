#include "relationalMachineModule.h"

struct RM_EditCallback:GraphEditCallback{
  RelationalMachineModule &RMM;
  RM_EditCallback(RelationalMachineModule &RMM):RMM(RMM){}
  virtual void cb_new(Item *it){
    LOG(-1) <<"NEWED: " <<*it;
//    Activity *act = newActivity(it);
//    if(act){
//      RMM.A().append(act);
//      LOG(2) <<"added activity '" <<*act <<"' for fact '" <<*it <<"'";
//    }else{
//      LOG(2) <<"Fact '" <<*it <<"' cannot be matched/created with an activity";
//    }
  }
  virtual void cb_delete(Item *it){
    LOG(-1) <<"DELETED: " <<*it;
    for(Activity *act:RMM.A()) if(act->fact==it){
      LOG(2) <<"removing activity '" <<*act <<"'";
      RMM.A().removeValue(act);
      delete act;
    }
  }
};

RelationalMachineModule::RelationalMachineModule():Module("RelationalMachineModule"){
}

RelationalMachineModule::~RelationalMachineModule(){
}

void RelationalMachineModule::open(){
  MT::open(fil,"z.RelationalMachineModule");
  RM.writeAccess();
  RM().init("machine.fol");
  RM().verbose = true;
  RM().state->callbacks.append(new RM_EditCallback(*this));
  RM.deAccess();
  threadStep(1);
}

void RelationalMachineModule::close(){
  listDelete(RM.set()->state->callbacks);
  fil.close();
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
  if(!effs.N && step_count) return; //on 1st iteration we need a step!

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
    it2act.setUni(NULL);
    LOG(2) <<"Syncing facts with activities..";
    for(Activity *act:A()){
      CHECK(act->fact == state(act->fact->index),"SOMETHING'S WRONG!");
      it2act(act->fact->index) = act;
    }

    A.writeAccess();
//    //-- del NULL activities
//    for(uint i=A().N; i--;){
//      Activity *act =A()(i);
//      if(act->fact==NULL){
//        LOG(2) <<"removing activity '" <<*act <<"'";
//        A().removeValue(act);
//        delete act;
//      }
//    }

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

