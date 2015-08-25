#include "RelationalMachineModule.h"

struct RM_EditCallback:GraphEditCallback{
  RelationalMachineModule &RMM;
  RM_EditCallback(RelationalMachineModule &RMM):RMM(RMM){}
  virtual void cb_new(Node *it){
    //LOG(-1) <<"NEWED: " <<*it;
//    Activity *act = newActivity(it);
//    if(act){
//      RMM.A().append(act);
//      LOG(2) <<"added activity '" <<*act <<"' for fact '" <<*it <<"'";
//    }else{
//      LOG(2) <<"Fact '" <<*it <<"' cannot be matched/created with an activity";
//    }
  }
  virtual void cb_delete(Node *it){
 //   LOG(-1) <<"DELETED: " <<*it;
    for(Activity *act:RMM.A()) if(act->fact==it){
 //     LOG(2) <<"removing activity '" <<*act <<"'";
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
  effects.writeAccess();
  MT::String effs = effects();
  effects().clear();
  effects.deAccess();
  fil <<RM.var->revision.getValue() <<endl <<"EFFECTS = " <<effs <<endl;
  if(!effs.N && step_count) return; //on 1st iteration we need a step!

  RM.writeAccess();
  A.writeAccess();
  if(effs.N){
    RM().applyEffect(effs);
    RM().fwdChainRules();
    fil <<"STATE = " <<RM().getState() <<endl <<"KB = " <<RM().getKB() << endl;
  }

  if(!step_count || effs.N){
    state.set() = RM().getState();

    //-- sync with activities: add activities for non-associated
    Graph &state = *RM().state;
    MT::Array<Activity*> fact2act(state.N);
    fact2act.setUni(NULL);
  //  LOG(2) <<"Syncing facts with activities..";
    for(Activity *act:A()){
      CHECK(act->fact == state(act->fact->index),"SOMETHING'S WRONG!");
      fact2act(act->fact->index) = act;
    }
    for(Node *it:state) if(fact2act(it->index)==NULL){ //fact is not associated yet
      Activity *act = newActivity(it);
      if(act){
        A().append(act);
     //   LOG(2) <<"added activity '" <<*act <<"' for fact '" <<*it <<"'";
      }else{
     //   LOG(2) <<"Fact '" <<*it <<"' cannot be matched/created with an activity";
      }
    }
  }
  A.deAccess();
  RM.deAccess();

  //TODO: cleanup? remove NULL facts from state?
}

