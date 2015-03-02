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
